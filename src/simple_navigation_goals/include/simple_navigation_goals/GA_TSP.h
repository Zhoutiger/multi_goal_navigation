#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <fstream>
#include <iostream>
#include<sstream>
#include<stdlib.h>
#include<time.h>
#include<math.h>
#include<vector>
#include<algorithm>
#include<unitree_guide/Command.h>

using namespace std;

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/
/********************************************************************************************************************************************/
// #define N 6					//种群大小（解个数）
// #define C 5					//城市个数
// #define T 5					//染色体基因个数（T=K+LV+1）
// #define cross_rate 0.75			//交叉率
// #define muta_rate 0.1			//变异率
// #define I 500					//迭代次数

const int N= 24;
const int C = 15;
const int T = 15;
const double cross_rate = 0.80;
const double muta_rate = 0.1;
const int I =5000;




move_base_msgs::MoveBaseGoal goal[C];

//10个城市的坐标：
// double citys_position[100][2] =
// {
// 	{1304,2312},{3639,1315},{4177,2244},{3712,1399},{3488,1535},
// 	{3326,1556},{3238,1229},{4196,1004},{4312,7900},{4386,570},
// 	{0,0}, {1024,1076}, {3451,987}, {521, 988}, {4509, 5342}
// };


double citys_position[15][4];

class Genetic_Algorithm
{
private:
	int i, j;
	int population[N][T];			//N-T,初始种群
	int it;							//迭代控制变量
	//int init_population[N][T];		//N-T,初始种群
	double F1[N];					//N，存放初始种群每个解方案（正）目标值
	double F2[N];					//N，存放变异种群每个解方案（正）目标值
	double R[N];					//N,存放每个解方案（倒）目标值
	double p[N];					//N，存放轮盘赌参考概率（每个解被选概率）
	int rand_population[N][T];		//N-T,轮盘赌种群
	int cross_population[N][T];		//N-T,交叉子代种群
	int muta_population[N][T];		//N-T,变异子代种群
	int mix_population[N][T];		//N-T,混合种群
	ros::NodeHandle nh;

public:
	/*《函数GA申明：遗传算法主体》*/
	void GA();
	/*《函数Initial_Population申明：生成初始种群》*/
	void Initial_Population();
	/*《函数Fitness申明：对传入解计算适应度值》*/
	double Fitness(int* input_solution);
	/*《distance函数声明：计算两两节点的距离，传入两个城市各自的坐标信息》*/
	double distance(double* city1, double* city2);
	/*《函数Selection申明：轮盘赌选出N个个体参与交叉》*/
	void Selection();
	/*《函数Crossover申明：概率交叉》*/
	void Crossover();
	/*《函数Order_Crossover申明：顺序交叉OX，并将交叉结果直接传给cross_population》*/
	void Order_Crossover(int* father, int* mother, int Z1, int Z2);
	/*《函数Mutation申明：概率变异》*/
	void Mutation();
	/*《函数Best_Solution定义：输出本次迭代搜索到的最好解，包括初始种群和变异种群》*/
	void Best_Solution();
	/*《函数Mixing_population声明：为下一迭代准备新的种群》
	构成：70%来自变异种群，20%来自初始种群，10%来自新产生个体。*/
	void Mixing_population();
};