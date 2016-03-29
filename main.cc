#include <iostream>
#include <libplayerc++/playerc++.h>

using namespace std;
using namespace PlayerCc;

int main(int argc, char** argv)
{
	PlayerClient pc("localhost",6665);
	Position2dProxy pp(&pc,0);
	//SonarProxy sp(&pc,0);
	LaserProxy lp(&pc);

	pp.SetMotorEnable(true);
	while (true)
	{
		pc.Read();

		int i;
		int sumLeft = 0;
		int sumRight = 0;
		bool isObsticle = false;
		int wantedYaw;
		for (i=0;i<222;i++)
		{
			sumLeft += lp[i + 444];
			sumRight += lp[i];
			if (lp[i + 222] < 0.8)
			{
				isObsticle = true;
			}
		}
		if (!isObsticle)
		{
			pp.SetSpeed(0.1,0);
		}
		else if (sumLeft < sumRight)
		{
			pp.SetSpeed(0,3.14/2);
			wantedYaw = pp.GetYaw() + 3.14/2;
			cout<< "Left!" <<endl;
		}
		else
		{
			pp.SetSpeed(0,-3.14/2);
			wantedYaw = pp.GetYaw() - 3.14/2;
			cout<< "Right!" <<endl;
		}


//		for (i=222;(i<444)&&(lp[i]>=0.8);i++)
//		{
//			pp.SetSpeed(0.1,0.5);
//
//				cout<<"Read "<<i+1<<":\n";
//				cout<<lp[i]<<endl;
//		}
//		if (i!=444){

//		}


	}


	pp.SetSpeed(0,0);
	return 0;
}
