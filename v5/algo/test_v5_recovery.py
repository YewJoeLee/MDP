from pathlib import Path
import sys, types, unittest, hashlib, py_compile
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'algo')); sys.path.insert(0,str(root/'rpi'))
sys.modules['serial']=types.SimpleNamespace(Serial=None)
import planner, config, rpi_stm_conn as runner, recovery_config as policy
policy.SETTLE_SECONDS=policy.FRAME_GAP_SECONDS=0
class Serial:
 def __init__(self, lines=()): self.lines=iter(lines); self.closed=False
 def readline(self): return next(self.lines,b'')
 def close(self): self.closed=True
class RecoveryTests(unittest.TestCase):
 def exercise(self, results, backup=True, fail_move=False, second=False):
  ser=Serial(); calls=[]; misses=[]; replans=[]; captures=[]
  def move(s,c,label): calls.append(c); return (False if fail_move else True),'DONE'
  runner.run_move=move
  def snap(oid,attempt,final): captures.append((oid,attempt,final)); return next(results,False)
  def replan(pose,ids): replans.append((pose,ids)); return {'status':'SUCCESS','segments':[dict(obstacle_id=2,hardware_commands=['FW2','SNAP2'],backup=None)]}
  seg=dict(obstacle_id=1,hardware_commands=['FW1','SNAP1'],backup={'command':'BW3','pose':[20,30,'N']} if backup else None)
  segments=[seg]+([dict(obstacle_id=2,hardware_commands=['FW99','SNAP2'])] if second else [])
  ok=runner.run_plan_v5({'segments':segments},snap,replan,misses.append,lambda:ser)
  self.assertTrue(ser.closed)
  return ok,calls,captures,misses,replans
 def test_terminal_backup_and_replan_flow(self):
  import io, contextlib
  output=io.StringIO()
  with contextlib.redirect_stdout(output):
   self.exercise(iter([False]*3+[True,True]),second=True)
  log=output.getvalue()
  for marker in ['[LEG]','[SETTLE]','| PRIMARY |','[BACKUP]','| BACKUP |','[REPLAN]','[REPLAN OK]','[NEW LEG]','[RUN COMPLETE]']:
   self.assertIn(marker,log)
  self.assertLess(log.index('[BACKUP]'),log.index('[REPLAN]'))
 def test_terminal_no_backup_and_miss(self):
  import io, contextlib
  output=io.StringIO()
  with contextlib.redirect_stdout(output):self.exercise(iter([]),False)
  self.assertIn('[BACKUP SKIPPED]',output.getvalue())
  self.assertIn('[NO IMAGE FOUND]',output.getvalue())
 def test_terminal_failed_move_stops(self):
  import io, contextlib
  output=io.StringIO()
  with contextlib.redirect_stdout(output):self.exercise(iter([]),fail_move=True)
  self.assertIn('[STOP]',output.getvalue())
  self.assertNotIn('[SNAP]',output.getvalue())
 def test_success_early(self):
  ok,m,c,f,r=self.exercise(iter([False,True])); self.assertTrue(ok); self.assertEqual(m,['FW1']); self.assertEqual(len(c),2); self.assertFalse(f)
 def test_six_frames_only_one_backup(self):
  ok,m,c,f,r=self.exercise(iter([])); self.assertTrue(ok); self.assertEqual(m,['FW1','BW3']);self.assertEqual(len(c),6);self.assertEqual(f,[1])
 def test_no_backup(self):
  ok,m,c,f,r=self.exercise(iter([]),False);self.assertEqual(len(c),3);self.assertEqual(m,['FW1']);self.assertEqual(f,[1])
 def test_replan_replaces_stale_leg(self):
  ok,m,c,f,r=self.exercise(iter([False]*3+[True,True]),second=True);self.assertTrue(ok);self.assertEqual(m,['FW1','BW3','FW2']);self.assertEqual(r,[([20,30,'N'],[2])])
 def test_motion_fault_no_capture(self):
  ok,m,c,f,r=self.exercise(iter([]),fail_move=True);self.assertFalse(ok);self.assertFalse(c)
 def test_done_failure(self):
  for status in ['STALL','TIMEOUT','IMUFAIL','ABORT','HEADINGERR']:
   ok,line=runner.wait_reply(Serial([f'DONE FW10 {status}\n'.encode()]),'FW10');self.assertFalse(ok)
 def test_stale_reply_ignored(self):
  ok,line=runner.wait_reply(Serial([b'DONE FW9 OK\n',b'DONE FW10 OK\n']),'FW10');self.assertTrue(ok);self.assertIn('FW10',line)
 def test_backup_geometry(self):
  obstacle=(1,10,10,'S');goal=(105,73,'N');g=planner.Geometry([obstacle])
  b=planner.backup_pose(obstacle,goal,g);self.assertEqual(b['command'],'BW5');self.assertEqual(b['sensor_distance_cm'],20)
  self.assertIsNone(planner.backup_pose(obstacle,(105,68,'N'),g))
  blocked=planner.Geometry([obstacle,(2,10,5,'N')]);self.assertIsNone(planner.backup_pose(obstacle,goal,blocked))
 def test_calibration_unchanged(self):
  import json
  baseline=json.loads((root/'algo/v4_baseline_hashes.json').read_text())
  self.assertEqual(hashlib.sha256((root/'algo/config.py').read_bytes()).hexdigest(),baseline['config_sha256'])
 def test_android_unchanged(self):
  import json
  baseline=json.loads((root/'algo/v4_baseline_hashes.json').read_text())
  for name,digest in baseline['android'].items():
   self.assertEqual(hashlib.sha256((root/'android'/name).read_bytes()).hexdigest(),digest,name)
 def test_python_compiles(self):
  for team in ['rpi','algo']:
   for p in (root/team).glob('*.py'):py_compile.compile(str(p),doraise=True)
if __name__=='__main__':unittest.main(verbosity=2)
