// #include "common/timeMarker.h"
// #include <unistd.h>

// //时间戳  微秒级， 需要#include <sys/time.h> 
// long long getSystemTime() {  
//     struct timeval t;  
//     gettimeofday(&t, NULL);
//     return 1000000 * t.tv_sec + t.tv_usec;  
// } 
// //时间戳  秒级， 需要getSystemTime()
// double getTimeSecond(){
//     double time = getSystemTime() * 0.000001;
//     return time;
// }

// //等待函数，微秒级，从startTime开始等待waitTime毫秒
// void absoluteWait(long long startTime, long long waitTime){
//     while(getSystemTime() - startTime < waitTime){
//         usleep(50);
//     }
// }