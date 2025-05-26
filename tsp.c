#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>


struct City {
	int id;
	float x;
	float y;
};

struct City city_data[1000000];
int city_data_num = 0;


double sqrt(double x)
{
  int i;
  double y, z, result;

  if(x == 0)
    return 0;
  else
    {
      y = 1;

      for(i = 0;i <= 10;i++)
	{
	  z = x / y;
	  result = (y + z) / 2;
	  y = result;
	}
      return result;
    }
}


/* This is the main function where TSP solver is implemented. 
   If you would like to use the other algorithm to solve TSP, 
   you can freely change this function. */
void do_travel(void)
{
	int i;
	int pid, cid;
	float px, py, cx, cy;
	float travel_distance = 0.0;
	
	/* set the initial city you visit */
	pid = city_data[city_data_num - 1].id;
	px = city_data[city_data_num - 1].x;
	py = city_data[city_data_num - 1].y;

	/* do traveling */
	for (i = 0 ; i < city_data_num; ++i) {
		/* set the current city you visit */
		cid = city_data[i].id;
		cx = city_data[i].x;
		cy = city_data[i].y;

		/* calculate distance between the previous city and the current one */
		travel_distance += (float)sqrt((cx - px) * (cx - px) + (cy - py) * (cy - py));

		/* print the status */
		printf("<%d>(%f, %f) => <%d>(%f, %f) : cumulative distance = %f\n", pid, px, py, cid, cx, cy, travel_distance);

		/* set the current city as the previous one */
		pid = cid;
		px = cx;
		py = cy;
	}
}


/* The program starts here */
int main(int argc, char** argv)
{
	char line[512];
	FILE* fp;

	/* check the number of arguments */
	if (argc != 2) {
		printf("usage: ./a.out input_file_name\n");
		exit(1);
	}

	/* check whether the file can be opened */
	fp = fopen(argv[1], "r");
	if (fp == NULL) {
		printf("The file (%s) cannot be opened!\n", argv[1]);
		exit(1);
	}
	
	/* read city data and write the position of each city into the array "city_data" */
	while(fgets(line, sizeof(line), fp)) {
		char* line_r;
		line_r = strtok(line, " \t\n");
		city_data[city_data_num].id = (int)strtol(line_r, NULL, 10);
		line_r = strtok(NULL, " \t\n");
		city_data[city_data_num].x = (float)strtod(line_r, NULL);
		line_r = strtok(NULL, " \t\n");
		city_data[city_data_num].y = (float)strtod(line_r, NULL);
		city_data_num++;
	}

	/* do traveling and calculate the distance */
	do_travel();
}