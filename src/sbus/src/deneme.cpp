// C++ program to demonstrate the
// use of flush function
#include <iostream>
#include <thread>
#include <chrono>
using namespace std;

int old[5] = {2, 3, 4, 5, 6};
int neww[5] = {2, 3, 4, 5, 6};
bool x;

int main()
{
	for (int i = 1; i < 5; ++i)
	{
		if (old[i] == neww[i])
		{
			x = true;
		}
		else{
			printf("old[%d] = %d and neww[%d] = %d",i , old[i], i, neww[i]);
			break;
		}
	}

	if (x == true)
	{
		printf("new array is equal to the old array \n");
	}

	return 0;
}
