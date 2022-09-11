/*改进的粒子群算法：结合GA和SA*/
#include<iostream>
using namespace std;
#include<stdlib.h>
#include<time.h>
#include<math.h>
#include<vector>
#include<algorithm>
#include <geometry_msgs/PoseStamped.h>

class Particle_Swarm_Optimization {
    public:
        Particle_Swarm_Optimization(vector<geometry_msgs::PoseStamped> poses) {
            for(auto pose : poses) {
                vector<double> goal;
                goal.push_back(pose.pose.position.x);
                goal.push_back(pose.pose.position.y);
                citys_position.push_back(goal);
            }

            T = citys_position.size();
            N = citys_position.size();
            C = citys_position.size();

            I = 200;

			population.resize(N, vector<int>(T));
			population_copy.resize(N, vector<int>(T));
			F.resize(N);
			F_copy.resize(N);
			Pbest.resize(N);
			Pbest_population.resize(N, vector<int>(T));
			Gbest_individual.resize(T);
			matrix.resize(N + 1, vector<double>(N + 1, INT_MAX));
		
        }
        Particle_Swarm_Optimization() {
			citys_position =
			{
				{1304,2312},{3639,1315},{4177,2244},{3712,1399},{3488,1535},
				{3326,1556},{3238,1229},{4196,1004},{4312,7900},{4386,570}
			};

            T = citys_position.size();
            N = citys_position.size();
            C = citys_position.size();

            I = 200;

			population.resize(N, vector<int>(T));
			population_copy.resize(N, vector<int>(T));
			F.resize(N);
			F_copy.resize(N);
			Pbest.resize(N);
			Pbest_population.resize(N, vector<int>(T));
			Gbest_individual.resize(T);
		}
    private:
        vector<vector<double>> citys_position;
        int N;  //种群大小（解个数）
        int C;  //城市个数
        int T;  //染色体基因个数（T=K+LV+1）
        int I;  //迭代次数

        double w = 0.1;					//惯性因子，视为变异率，如果为0，则不进行该变异操作
        double c1 = 2;					//自身认知能力，如果为0，则不进行该交叉操作
        double c2 = 2;					//群体认知能力，如果为0，则不进行该交叉操作
        double Lower = 0.35;				//r1,r2区间下限
        double Upper = 0.45;				//r1,r2区间上限
        double	Accept = 0.5;				//粒子群更新时接受差解的概率

private:
	int i, j;
	int it;								//迭代变量
	//int population[N][T];				//粒子群
    vector<vector<int>> population;
	//int population_copy[N][T];			//粒子群copy，用于处理交叉、变异
    vector<vector<int>> population_copy;
	//double F[N];						//粒子群适应度值
    vector<double> F;
	//double F_copy[N];					//population_copy适应度值
    vector<double> F_copy;
	//double Pbest[N];					//粒子群每个个体至今最好值
    vector<double> Pbest;
	//int Pbest_population[N][T];			//粒子群每个个体至今最好解
    vector<vector<int>> Pbest_population;
	double Gbest;						//粒子群至今搜到的最好值
	//int Gbest_individual[T];			//粒子群至今搜到的最好解
    vector<int> Gbest_individual;

	vector<vector<double>> matrix;
public:
	void set_matrix(int row, int col, double val) {
		matrix[row][col] = val;
	}

	/*《函数Init_Values申明：初始化一些变量和数组》*/
	void Init_Values();
	/*《函数PSO声明：粒子群算法主体》*/
	vector<int> PSO();
	/*《函数Initial_Population申明：生成初始种群》*/
	void Initial_Population();
	/*《函数Fitness申明：对传入解计算适应度值》*/
	double Fitness(vector<int> input_solution);
	/*《distance函数声明：计算两两节点的距离，传入两个城市各自的坐标信息》*/
	double distance(vector<double> city1, vector<double> city2);
	double distance(int index1, int index2) { return matrix[index1][index2];}
	/*《函数Update_Best申明：更新Pbest,Gbest,以及相应的解》*/
	void Update_Best();
	/*《函数Update_population申明：根据速度公式更新粒子群每个个体的位置》*/
	void Update_population();
	/*《函数Pbest_Crossover声明：粒子群个体与自身当前最优概率交叉》*/
	void Pbest_Crossover();
	/*《函数Order_Crossover申明：顺序交叉OX，直接对population_copy操作》*/
	void Order_Crossover(vector<int> father, vector<int> mother, int k);
	/*《函数Gbest_Crossover声明：粒子群个体与全局最优概率交叉》*/
	void Gbest_Crossover();
	/*《函数Mutation声明：对两次交叉后的population_copy中个体进行简单的两点变异》*/
	void Mutation();
};


// int main()
// {
// 	srand((unsigned)time(NULL));

// 	Particle_Swarm_Optimization PSO1;
// 	PSO1.Init_Values();
// 	PSO1.PSO();

// 	//system("pause");
// 	return 0;
// }
/*《函数Init_Values申明：初始化一些变量和数组》*/
void Particle_Swarm_Optimization::Init_Values()
{
	//初始化Pbest[N]
	for (i = 0; i < N; i++)
	{
		Pbest[i] = 100000000;
	}
	//初始化Gbest
	Gbest = 100000000;
}
/*《函数PSO声明：粒子群算法主体》*/
vector<int> Particle_Swarm_Optimization::PSO()
{
	/*①《调用函数Initial_Population：生成初始种群》*/
	Initial_Population();
	/*②《调用函数Fitness：对粒子种群population传入解计算适应度值》*/
	//cout << "Initial sucess" <<endl;
	for (i = 0; i < N; i++)
	{
		F[i] = Fitness(population[i]);
	}
	/*迭代开始*/
	for (it = 0; it < I; it++)
	{
		/*③《调用函数Update_Best：更新Pbest,Gbest,以及相应的解》*/
		Update_Best();
		/*④《函数Update_population：根据速度公式更新粒子群每个个体的位置》*/
		Update_population();
		/*⑤《函数Fitness：计算population_copy适应度值》*/
		for (i = 0; i < N; i++)
		{
			F_copy[i] = Fitness(population_copy[i]);
		}
		/*⑥以上只是对population_copy作位置改变，还需以一定的概率传给原粒子群population*/
		for (i = 0; i < N; i++)
		{
			if (F_copy[i] < F[i])
			{
				//一定接受好的解
				for (j = 0; j < T; j++)
				{
					population[i][j] = population_copy[i][j];
					F[i] = F_copy[i];
				}
			}
			else
			{
				//概率接受差的解
				double r = (double)rand() / RAND_MAX;
				if (r <= Accept)
				{
					for (j = 0; j < T; j++)
					{
						population[i][j] = population_copy[i][j];
						F[i] = F_copy[i];
					}
				}
			}
		}

	}
	return Gbest_individual;
}
/*《函数Initial_Population定义：生成初始种群》*/
void Particle_Swarm_Optimization::Initial_Population()
{
	//①一个城市序列
	vector<int> temp_city;
	for (int i = 0; i < C; i++)
	{
		temp_city.push_back(i + 1);
	}
	//②打乱后生成初始种群
	for (i = 0; i < N; i++)
	{
		random_shuffle(temp_city.begin(), temp_city.end());
		for (int j = 0; j < temp_city.size(); j++)
		{
			population[i][j] = temp_city[j];
		}
	}
	//③初始化population_copy
	for (i = 0; i < N; i++)
	{
		for (j = 0; j < T; j++)
		{
			population_copy[i][j] = population[i][j];
		}
	}
	cout << "init_population:" << endl;
	for (i = 0; i < N; i++)
	{
		for (j = 0; j < T; j++)
		{
			cout << population[i][j] << " ";
		}
		cout << endl;
	}
	cout << endl;
}
/*《Fitness函数定义：对传入解计算适应度值》*/
double Particle_Swarm_Optimization::Fitness(vector<int>  input_solution)
{
	//①初始化路径长度
	double cost = 0;
	//②从第一个城市遍历到最后一个城市
	for (int j = 0; j < C - 1; j++)
	{
		/*《调用distance函数：计算两两节点的距离，传入两个城市各自的坐标信息》*/
		//cost += distance(citys_position[input_solution[j] - 1], citys_position[input_solution[j + 1] - 1]);
		cost += distance(input_solution[j], input_solution[j + 1]);
	}
	//③从第一个城市回到第一个城市
	cost += distance(0, input_solution[0]);

	return cost;
}
/*《distance函数定义：计算两两节点的距离，传入两个城市各自的坐标信息》*/
double Particle_Swarm_Optimization::distance(vector<double> city1, vector<double> city2)
{
	/*计算相邻两城市间距离*/
	double x1 = city1[0];			//城市1横坐标
	double y1 = city1[1];
	double x2 = city2[0];
	double y2 = city2[1];			//城市2纵坐标
	double dist = pow((pow((x1 - x2), 2) + pow((y1 - y2), 2)), 0.5);

	return dist;					//返回距离值
}
/*《函数Update_Best定义：更新Pbest,Gbest,以及相应的解》*/
void Particle_Swarm_Optimization::Update_Best()
{
	//①先根据粒子群适应度更新每个个体的最优值、最优解
	for (i = 0; i < N; i++)
	{
		if (F[i] < Pbest[i])
		{
			//更新个体最优值
			Pbest[i] = F[i];		
			//更新个体最优解
			for (j = 0; j < T; j++)
			{
				Pbest_population[i][j] = population[i][j];
			}
		}
	}
	//②再根据各个个体的最好值，来更新全局的最优值和最优解
	for (i = 0; i < N; i++)
	{
		if (Pbest[i] < Gbest)
		{
			//更新全局最优值
			Gbest = Pbest[i];
			//更新全局最优解
			for (j = 0; j < T; j++)
			{
				Gbest_individual[j] = Pbest_population[i][j];
			}
		}
	}
	//③输出本次迭代的全局最好解及其适应度值
	if(it == I - 1) {
		cout << "第" << it << "次迭代最好方案：";
		cout << "0 " <<endl;
		for (j = 0; j < T; j++)
		{
			cout << "-->" << Gbest_individual[j] ;
		}
		cout << "适应度值：" << Gbest << endl << endl;
	}
}
/*《函数Update_population申明：根据速度公式更新粒子群每个个体的位置》*/
void Particle_Swarm_Optimization::Update_population()
{
	/*
	速度更新公式新定义：
	v[i]= w*v[i] + c1*r1(0.35,0.45)(pbest[i]-x[i]) + c1*r2(0.35,0.45)(gbest-x[i]);
	x[i+1]=x[i]+v[i];
	其中：w视为变异率，c1*r1视为与自身最优交叉率，c2*r2视为与全局最优交叉率。*/
	
	if (c1 != 0)
	{
		/*①《调用函数Pbest_Crossover：粒子群个体与自身当前最优概率交叉》*/
		Pbest_Crossover();
	}
	if (c2 != 0)
	{
		/*②《调用函数Gbest_Crossover声明：粒子群个体与全局最优概率交叉》*/
		Gbest_Crossover();
	}
	/*③《调用函数Mutation：对两次交叉后的population_copy中个体进行简单的两点变异》*/
	Mutation();
}
/*《函数Pbest_Crossover定义：粒子群个体与自身当前最优概率交叉》*/
void Particle_Swarm_Optimization::Pbest_Crossover()
{
	for (i = 0; i < N; i++)
	{
		//①随机生成r1(0.35,0.45):rand() / (double)RAND_MAX *(Upper - Lower) + Lower
		double r1 = rand() / (double)RAND_MAX *(Upper - Lower) + Lower;
		//②个体最优交叉率
		double Pbest_CrossRate = c1 * r1;
		//③概率交叉
		double r = (double)rand() / RAND_MAX;
		if (r <= Pbest_CrossRate)
		{
			/*《调用函数Order_Crossover：顺序交叉OX，直接对population_copy操作》*/
			Order_Crossover(population_copy[i], Pbest_population[i], i);
		}
		else
		{
			/*保持population_copy[i]不变化就行*/
		}
	}
}
/*《函数Order_Crossover定义：顺序交叉OX，直接对population_copy操作》*/
void Particle_Swarm_Optimization::Order_Crossover(vector<int> father, vector<int> mother, int k)
{
	int cut_point1, cut_point2;
	int child_indiv1[T] = { 0 };	//T，交叉子代1
	//①随机生成交叉点
	cut_point1 = rand() % (T - 1);	//cutpoint[5]={0,1..T-3}
	cut_point2 = rand() % T;		//cutpoint[5]={0,1,2..T-2}
	while (cut_point1 >= cut_point2)
	{
		cut_point2 = rand() % T;	//cut_point2>cut_point1
	}
	//②先取出交叉段中间部分的基因
	for (int x = cut_point1; x <= cut_point2; x++)
	{
		child_indiv1[x] = father[x];		 //子代1取父代的中段基因
	}
	/*如果第一交换点不是在开头*/
	if (cut_point1 != 0)
	{
		//③先对子代1取首尾段基因
		int index1 = 0;							 //子代1的索引
		for (int y = 0; y < T; y++)				 //搜索母代基因
		{
			bool bt = true;
			for (int z = cut_point1; z <= cut_point2; z++)
			{
				//搜索子代中段基因对比
				if (mother[y] == child_indiv1[z])
				{
					bt = false;					 //改变bool值，使其不执行后续代码
					//搜到重复则跳出当前最近for循环，执行以下，但是bt=false，故下面程序不会被执行！
					break;
				}
			}
			//经过以上对比，若bool值没有改变，即该基因是新基因，则赋给子代
			if (bt == true)
			{
				child_indiv1[index1] = mother[y];//母代赋值给子代1
				index1 += 1;					 //索引子代下一个基因
				//如果首段子代基因赋值完毕，则跳到尾段基因赋值部分
				if (index1 == cut_point1)
				{
					//如果第二交换点不在末尾，那么还有第三段基因
					if (cut_point2 != T - 1)
					{
						index1 = cut_point2 + 1;
					}
					//第二交换点正好在末尾，那么退出
					else
					{
						break;
					}
				}
			}
		}
		//⑤将交叉子代结果传给population_copy
		for (j = 0; j < T; j++)
		{
			population_copy[k][j] = child_indiv1[j];
		}
		
	}
	/*如果首交换点在开头*/
	else
	{
		/*如果第二交换点正好也在尾部，那么就不用交叉了*/
		if (cut_point2 == T - 1)
		{
			//⑤将父代们直接传给population_copy
			for (j = 0; j < T; j++)
			{
				population_copy[k][j] = father[j];
			}
		}
		/*如果第二交换点不在尾部*/
		else
		{
			//③先对子代1取第二段基因
			int index1 = cut_point2 + 1;			 //子代1的索引
			for (int y = 0; y < T; y++)				 //搜索母代基因
			{
				bool bt = true;
				for (int z = cut_point1; z <= cut_point2; z++)
				{
					//搜索子代中段基因对比
					if (mother[y] == child_indiv1[z])
					{
						bt = false;					 //改变bool值，使其不执行后续代码
						//搜到重复则跳出当前最近for循环，执行以下，但是bt=false，故下面程序不会被执行！
						break;
					}
				}
				//经过以上对比，若bool值没有改变，即该基因是新基因，则赋给子代
				if (bt == true)
				{
					child_indiv1[index1] = mother[y];//母代赋值给子代1
					index1 += 1;					 //索引子代下一个基因
					/*如果下一个索引没超界*/
					if (index1 != T)
					{
						index1 += 0;
					}
					/*如果下一个索引超界了，则退出*/
					else
					{
						break;
					}
				}
			}
			//⑤将交叉子代结果传给cross_population
			for (j = 0; j < T; j++)
			{
				population_copy[k][j] = child_indiv1[j];
			}
		}
	}

}
/*《函数Gbest_Crossover定义：粒子群个体与全局最优概率交叉》*/
void Particle_Swarm_Optimization::Gbest_Crossover()
{
	for (i = 0; i < N; i++)
	{
		//①随机生成r2(0.35,0.45):rand() / (double)RAND_MAX *(Upper - Lower) + Lower
		double r2 = rand() / (double)RAND_MAX *(Upper - Lower) + Lower;
		//②个体最优交叉率
		double Gbest_CrossRate = c2 * r2;
		//③概率交叉
		double r = (double)rand() / RAND_MAX;
		if (r <= Gbest_CrossRate)
		{
			/*《调用函数Order_Crossover：顺序交叉OX，直接对population_copy操作》*/
			Order_Crossover(population_copy[i], Gbest_individual, i);
		}
		else
		{
			/*保持population_copy[i]不变化就行*/
		}
	}


}
/*《函数Mutation定义：对两次交叉后的population_copy中个体进行简单的两点变异》*/
void Particle_Swarm_Optimization::Mutation()
{
	double MutaRate = w;
	//变异算子
	int point1, point2;					//变异点
	for (i = 0; i < N; i++)
	{
		double r = { 0.0 };
		r = (double)rand() / RAND_MAX;	//随机一个变异率
		//变异
		if (r <= MutaRate)
		{
			point1 = rand() % T;
			point2 = rand() % T;
			while (point1 == point2)
			{
				point2 = rand() % T;	//避免交换点重复
			}
			swap(population_copy[i][point1], population_copy[i][point2]);
		}
	}

}