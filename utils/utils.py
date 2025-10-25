import datetime

def printLog(info):
    timeNow = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S   ')
    print(timeNow + info)
    return timeNow + info

def timeNow2Str():
    return datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S   ')