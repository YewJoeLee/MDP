"""Host regression test of the actual wrapper/helpers extracted from latest_stm.c.
Run: python test_turn_compensation.py (requires gcc on PATH). No hardware used.
"""
from pathlib import Path
import subprocess, tempfile, shutil
source = Path(__file__).with_name('latest_stm.c').read_text()
def function(signature):
    start = source.index(signature)
    brace = source.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]
helper = function('static const char *Turn_Status(') + '\n' + function('static void Turn_CompensationFailed(')
wrapper = function('float Car_Turn_Square(')
harness = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#define TURN_LEFT 1
#define TURN_COMP_NOW(i) (comp_values[i])
static int comp_values[4],turn_failed,drive_reason,turn_reason,drives,turns,brakes,delays;
static int drive_sign,last_comp;
static float turn_brake_rate,imu_max_dps;
static unsigned imu_err_count,imu_sat_count;
static char events[16];static int event_count;
static void event(char c){events[event_count++]=c;events[event_count]=0;}
static void Motor_Brake(void){brakes++;event('B');}
static int Drive_Distance(int32_t cm,int dir){drives++;last_comp=cm;drive_sign=dir;event('D');return drive_reason;}
static void osDelay(int ms){assert(ms==100);delays++;}
'''
harness += helper
harness += r'''
static float Car_Turn(float target,int direction,int dir){
 assert(target>0);assert(direction==1 || direction==-1);assert(dir==1 || dir==-1);
 turns++;event('T');turn_failed=turn_reason;return 89.5f;
}
'''
harness += wrapper
harness += r'''
static void reset(void){
 for(int i=0;i<4;i++)comp_values[i]=7+i;
 drive_reason=turn_reason=drives=turns=brakes=delays=0;
 event_count=0;events[0]=0;turn_failed=4;
 turn_brake_rate=99;imu_max_dps=99;imu_err_count=99;imu_sat_count=99;
}
int main(void){
 int checks=0;
 const char *expected[]={"OK","TIMEOUT","STALL","IMUFAIL","ABORT"};
 for(int direction=-1;direction<=1;direction+=2){
  for(int reason=1;reason<=4;reason++){
   reset();drive_reason=reason;
   assert(Car_Turn_Square(90,direction,-1)==0);
   assert(drives==1 && turns==0 && brakes==1 && delays==0);
   assert(strcmp(events,"DB")==0 && drive_sign==1);
   assert(strcmp(Turn_Status(),expected[reason])==0);
   assert(turn_brake_rate==0 && imu_max_dps==0 && imu_err_count==0 && imu_sat_count==0);
   checks++;
   reset();drive_reason=reason;
   assert(Car_Turn_Square(90,direction,1)==89.5f);
   assert(drives==1 && turns==1 && brakes==1 && drive_sign==-1);
   assert(strcmp(events,"TDB")==0);
   assert(strcmp(Turn_Status(),expected[reason])==0);checks++;
  }
  for(int reason=1;reason<=3;reason++){
   reset();turn_reason=reason;
   Car_Turn_Square(90,direction,1);
   assert(drives==0 && turns==1 && brakes==1 && delays==0);
   assert(turn_failed==reason && strcmp(events,"TB")==0);checks++;
   reset();turn_reason=reason;
   Car_Turn_Square(90,direction,-1);
   assert(drives==1 && turns==1 && brakes==1);
   assert(turn_failed==reason && strcmp(events,"DTB")==0);checks++;
  }
  for(int dir=-1;dir<=1;dir+=2){
   reset();Car_Turn_Square(90,direction,dir);
   assert(drives==1 && turns==1 && brakes==0 && turn_failed==0);
   assert(strcmp(events,dir<0?"DT":"TD")==0);
   assert(last_comp==comp_values[(dir<0?2:0)+(direction==TURN_LEFT?0:1)]);checks++;
   reset();for(int i=0;i<4;i++)comp_values[i]=0;
   Car_Turn_Square(90,direction,dir);
   assert(drives==0 && turns==1 && turn_failed==0);checks++;
   reset();Car_Turn_Square(30,direction,dir);
   assert(drives==0 && turns==1 && turn_failed==0);checks++;
  }
 }
 printf("%d compensation sequencing/status cases passed\n",checks);
 return 0;
}
'''
compiler=shutil.which('gcc')
if not compiler: raise SystemExit('gcc required for host wrapper test')
with tempfile.TemporaryDirectory(prefix='v5_compensation_') as temp:
    c=Path(temp)/'test.c';exe=Path(temp)/'test.exe';c.write_text(harness)
    subprocess.run([compiler,'-std=c99','-Wall','-Wextra','-Werror',str(c),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
