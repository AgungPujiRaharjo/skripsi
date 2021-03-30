from datetime import datetime

#----------------pewaktuan---------------------------------
def wait(second):
    micSecond_input=second*1000000
    old_time=datetime.now()
    old_timeMicros=(old_time.minute*60000000)+(old_time.second*1000000)+old_time.microsecond
    interval=0

    while interval<micSecond_input:
        current_time=datetime.now()
        current_timeMicros=(current_time.minute*60000000)+(current_time.second*1000000)+current_time.microsecond
        interval=current_timeMicros-old_timeMicros
        # print("interval",interval)

def micros():
    old_time=datetime.now()
    old_timeMicros=(old_time.minute*60000000)+(old_time.second*1000000)+old_time.microsecond
    # print(old_timeMicros)
    return old_timeMicros
#----------------------------------------------------------
