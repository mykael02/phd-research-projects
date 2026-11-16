import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from balance import simulate
from run_robustness import outcome, evaluate, run_grid, summarize, boundary_candidates, audit_boundaries, save_report

CONFIG=json.loads((ROOT/'config.json').read_text())


class RobustnessTests(unittest.TestCase):
    def test_outcomes_keep_unsettled_separate(self):
        self.assertEqual(outcome({'fallen':True,'recovery_s':None}),'fallen')
        self.assertEqual(outcome({'fallen':False,'recovery_s':None}),'unsettled')
        self.assertEqual(outcome({'fallen':False,'recovery_s':0.0}),'recovered')

    def test_small_grid_complete_and_repeatable(self):
        a=run_grid(CONFIG,tilts=(0,10),pushes=(0,30),torques=(.1,))
        b=run_grid(CONFIG,tilts=(0,10),pushes=(0,30),torques=(.1,))
        self.assertEqual(a,b);self.assertEqual(len(a),8)
        self.assertEqual(len({(r['controller'],r['initial_tilt_deg'],r['push_force_n'],r['torque_limit_nm']) for r in a}),8)
        for summary in summarize(a):
            self.assertEqual(summary['tested'],4)
            self.assertEqual(sum(summary[k] for k in ('recovered','unsettled','fallen')),4)

    def test_pre_push_fall_not_mislabeled_as_exposure(self):
        r=evaluate(CONFIG,'pid',20,30,.1)
        self.assertTrue(r['fallen']);self.assertFalse(r['push_started']);self.assertFalse(r['push_completed'])
        r=evaluate(CONFIG,'lqr',5,5,.7)
        self.assertTrue(r['push_started']);self.assertTrue(r['push_completed'])
        self.assertFalse(evaluate(CONFIG,'lqr',0,0,.7)['push_started'])

    def test_detects_nonmonotone_boundaries(self):
        rows=[dict(controller='pid',initial_tilt_deg=0,push_force_n=i,torque_limit_nm=.7,outcome=status)
              for i,status in enumerate(('recovered','fallen','recovered'))]
        self.assertEqual(boundary_candidates(rows),rows)

    def test_physics_refinement_preserves_control_sampling(self):
        a,rows_a=simulate('lqr',CONFIG,push_force=3,physics_substeps=1)
        b,rows_b=simulate('lqr',CONFIG,push_force=3,physics_substeps=2)
        self.assertEqual([r['time'] for r in rows_a],[r['time'] for r in rows_b])
        self.assertEqual([r['push'] for r in rows_a],[r['push'] for r in rows_b])
        self.assertEqual(outcome(a),outcome(b))
        for value in (0,-1,1.5,True):
            with self.assertRaises(ValueError):simulate('lqr',CONFIG,physics_substeps=value)

    def test_report_keeps_data_and_replay_link(self):
        records=run_grid(CONFIG,tilts=(0,),pushes=(0,30),torques=(.7,))
        audit=audit_boundaries(CONFIG,records,limit=2)
        with tempfile.TemporaryDirectory() as directory:
            result=save_report(directory,CONFIG,records,audit)
            report=(Path(directory)/'report.html').read_text(encoding='utf-8')
            saved=json.loads((Path(directory)/'results.json').read_text())
            self.assertEqual(saved,result);self.assertEqual(saved['records'],records)
            self.assertNotIn('DATA_PAYLOAD',report);self.assertNotIn('REPLAY_URL_JSON',report)
            self.assertIn('index.html',report);self.assertIn('study-data',report)

    def test_invalid_grid(self):
        for options in ({'tilts':()}, {'tilts':(0,0)}, {'pushes':(float('nan'),)}, {'torques':(0,)}, {'tilts':(45,)}):
            with self.assertRaises(ValueError):run_grid(CONFIG,**options)


if __name__=='__main__':unittest.main()
