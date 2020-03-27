
#include "basic_define/bucket_queue.h"
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>


std::vector<int> BucketPrioQueue::sqrIndices;
int BucketPrioQueue::numBuckets;


BucketPrioQueue::BucketPrioQueue() {
  // make sure the index array is created
  if (sqrIndices.size()==0) initSqrIndices();
  nextBucket = INT_MAX;

  buckets = std::vector<std::queue<Vec2i> >(numBuckets);

  // reset element counter 
  count = 0;
}

bool BucketPrioQueue::empty() {
  return (count==0);
}

void BucketPrioQueue::push(int prio, Vec2i t) {
  if (prio>=(int)sqrIndices.size()) {
    fprintf(stderr, "error: priority %d is not a valid squared distance x*x+y*y, or x>MAXDIST or y>MAXDIST.\n", prio);
    exit(-1);
  }
  int id = sqrIndices[prio]; //id取值在[0, 2*MAXDIST^2]之间
  if (id<0) {
    fprintf(stderr, "error: priority %d is not a valid squared distance x*x+y*y, or x>MAXDIST or y>MAXDIST.\n", prio);
    exit(-1);
  }
  buckets[id].push(t);//buckets[id]为同一个pri的元素集合
  //    printf("pushing %d with prio %d into %d. Next: %d\n", t.x, prio, id, nextBucket);
  // nextBucket倾向于越来越小，即vector中靠前的元素，优先级更高
  if (id<nextBucket) nextBucket = id;
  //    printf("push new next is %d\n", nextBucket);
  count++;
}

Vec2i BucketPrioQueue::pop() {
  int i;
  assert(count>0);
  for (i = nextBucket; i<(int)buckets.size(); i++) {
    if (!buckets[i].empty()) break;	//找到最小距离的一串数，跳出循环
  }
  assert(i<(int)buckets.size());//确保在有效范围
  nextBucket = i;
  count--;
  Vec2i p = buckets[i].front();//取出队首元素
  buckets[i].pop();//从队列移除队头元素
  return p;
}

void BucketPrioQueue::initSqrIndices() {
  //MAXDIST=1000 是根据什么设置的？
  sqrIndices = std::vector<int>(2*MAXDIST*MAXDIST+1, -1);

  int count=0;
  for (int x=0; x<=MAXDIST; x++) {
    for (int y=0; y<=x; y++) {  //为什么 y<=x？
      int sqr = x*x+y*y;        //sqr的最大值为 MAXDIST^2 + MAXDIST^2 = 2*MAXDIST^2
      sqrIndices[sqr] = count++;
    }
  }
  numBuckets = count;
}
