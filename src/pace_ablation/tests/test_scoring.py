"""Compile the actual scoring methods without ROS and check ablation semantics."""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[3]
ORIGINAL = ROOT / 'src/fuzzy_decision/src/fuzzy_decision_node.cpp'
ABLATION = ROOT / 'src/pace_ablation/src/feature_ablation_node.cpp'
METHODS = ('computeMembership', 'computeSemanticMembership', 'computeDistanceMembership',
           'computeExplorationGainMembership', 'fuzzyInference', 'defuzzify')


def method(source, name):
    match = re.search(r'^    (?:std::map<std::string, double>|double|void) ' + name + r'\(',
                      source, re.MULTILINE)
    if not match:
        raise AssertionError(f'Missing scoring method: {name}')
    start = source.index('{', match.start())
    depth = 1
    end = start + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


class ScoringTests(unittest.TestCase):
    def test_original_rule_functions_are_unchanged(self):
        old, new = ORIGINAL.read_text(), ABLATION.read_text()
        for name in METHODS:
            self.assertEqual(method(old, name), method(new, name), name)
        self.assertEqual(method(old, 'weightsCallback'), method(new, 'weightsCallback'))

    @unittest.skipUnless(shutil.which('g++'), 'g++ is needed for the scoring check')
    def test_score_equivalence_information_removal_and_reference_mean(self):
        source = ABLATION.read_text()
        functions = '\n'.join(method(source, name) for name in METHODS)
        evaluation = method(source, 'evaluateFeatureAblation')
        header = '''
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <map>
#include <random>
#include <string>
#include <vector>
struct Scorer {
    double weight_extremely_high_=10, weight_high_=8, weight_medium_=5, weight_low_=2;
    std::string ablation_mode_="full";
    std::vector<double> rl_weights_={.25,.25,.25,.25};
'''
        main = r'''
};
double base(Scorer& s, std::array<double,4> x, bool usable) {
    auto out=s.fuzzyInference(s.computeSemanticMembership(x[0]),
        s.computeDistanceMembership(x[1]), s.computeExplorationGainMembership(x[2]),
        s.computeMembership(x[3]),s.rl_weights_);
    if (!usable) {
        out["极高"]*=.60; out["高"]*=.80; out["低"]=std::max(out["低"],.25);
    }
    return s.defuzzify(out);
}
double score(Scorer& s, const std::array<double,4>& x, bool usable) {
    std::map<std::string,double> out;
    return s.evaluateFeatureAblation(x[0],x[1],x[2],x[3],usable,out);
}
int main() {
    Scorer s;
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> uniform(0,1);
    const std::array<std::string,4> names={"semantic","distance","gain","risk"};
    for (bool usable:{true,false}) {
        std::array<bool,4> responds={false,false,false,false};
        std::array<double,4> first={-1,-1,-1,-1};
        for(int trial=0;trial<200;++trial) {
            std::array<double,4> x={uniform(rng),uniform(rng),uniform(rng),uniform(rng)};
            // Exercise nonuniform policy outputs: a hardcoded fixed-weight
            // scorer must not accidentally pass the full-PACE parity check.
            double total=0;
            for(double& weight:s.rl_weights_) { weight=uniform(rng); total+=weight; }
            for(double& weight:s.rl_weights_) weight/=total;
            s.ablation_mode_="full";
            assert(std::abs(score(s,x,usable)-base(s,x,usable))<1e-12);
            for(size_t axis=0;axis<4;++axis) {
                s.ablation_mode_=names[axis];
                const double actual=score(s,x,usable);
                assert(std::isfinite(actual) && actual>=0 && actual<=10);
                double expected=0;
                for(double ref:{0.,.25,.5,.75,1.}) {
                    auto changed=x; changed[axis]=ref;
                    expected+=base(s,changed,usable)/5.;
                    assert(std::abs(score(s,changed,usable)-actual)<1e-12);
                }
                assert(std::abs(actual-expected)<1e-12);
                if(trial==0) first[axis]=actual;
                else if(std::abs(actual-first[axis])>1e-6) responds[axis]=true;
            }
        }
        for(bool responds_to_remaining_features:responds) assert(responds_to_remaining_features);
    }
}
'''
        with tempfile.TemporaryDirectory(prefix='pace-score-check-') as tmp:
            code, executable = Path(tmp) / 'check.cpp', Path(tmp) / 'check'
            code.write_text(header + functions + '\n' + evaluation + main)
            subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-pedantic',
                            str(code), '-o', str(executable)], check=True)
            subprocess.run([str(executable)], check=True)


if __name__ == '__main__':
    unittest.main()
