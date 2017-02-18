#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "CoreSLAM.h"

#define TEST_FILENAME "test_lab.dat"
#define TEST_SCAN_SIZE 682
#define TEST_MIN_DIST 20
#define TEST_ANGLE_MIN -120
#define TEST_ANGLE_MAX +120
#define TEST_OFFSET_LASER 145

#define TEST_HOLE_WIDTH 600

ts_map_t trajectory, map;


//记录地图，记录全局地图，把机器人走过路径上的栅格置为0 ，有障碍为0
void record_map(ts_map_t *map, ts_map_t *overlay, char *filename, int width, int height)  //width和height = TS_MAP_SIZE
{
    int x, y, xp, yp;
    FILE *output;
    output = fopen(filename, "wt");
    fprintf(output, "P2\n%d %d 255\n", width, height);
    y = (TS_MAP_SIZE - height) / 2;
    for (yp = 0; yp < height; y++, yp++)
	{
        x = (TS_MAP_SIZE - width) / 2; 
		for (xp = 0; xp < width; x++, xp++) 
		{
		    if (overlay->map[ (TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x] == 0)    //如果overlay路径地图中机器人走过耨个栅格，则这个栅格值输出0
                fprintf(output, "0 ");
            else                                                                //否则，输出全局地图map中的值
                fprintf(output, "%d ", (int)(map->map[ (TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x]) >> 8);
		}
		fprintf(output, "\n");
    }
    fclose(output);
}

void draw_scan(ts_scan_t *scan, ts_map_t *map, ts_position_t *pos)    //把机器人扫描的雷达点转化到全局地图中，栅格置为0，TS_OBSTACLE
{
    double c, s;
    double x2p, y2p;
    int i, x1, y1, x2, y2;

    c = cos(pos->theta * M_PI / 180);
    s = sin(pos->theta * M_PI / 180);
    x1 = (int)floor(pos->x * TS_MAP_SCALE + 0.5);
    y1 = (int)floor(pos->y * TS_MAP_SCALE + 0.5);
    // Translate and rotate scan to robot position
    for (i = 0; i != scan->nb_points; i++) {
        if (scan->value[i] != TS_NO_OBSTACLE) {
            x2p = c * scan->x[i] - s * scan->y[i];
            y2p = s * scan->x[i] + c * scan->y[i];
            x2p *= TS_MAP_SCALE;
            y2p *= TS_MAP_SCALE; 
            x2 = (int)floor(pos->x * TS_MAP_SCALE + x2p + 0.5);
            y2 = (int)floor(pos->y * TS_MAP_SCALE + y2p + 0.5);
            if (x2 >= 0 && y2 >= 0 && x2 < TS_MAP_SIZE && y2 < TS_MAP_SIZE)
                map->map[y2 * TS_MAP_SIZE + x2] = 0;
        }
    }
}

ts_position_t monte_carlo_move(ts_scan_t *scan, ts_map_t *map, ts_position_t *start_pos, int debug)
//scan: 这一帧的雷达数据;    map: 全局地图；  start_pos:上一时刻的机器人位置；   debug:
{
    ts_position_t cpp, currentpos, bestpos, lastbestpos;
    int currentdist;
    int bestdist, lastbestdist;
    int counter = 0;

    currentpos = bestpos = lastbestpos = *start_pos;
    currentdist = ts_distance_scan_to_map(scan, map, &currentpos);    //sum计算这一帧每个雷达点转化到全局地图中所在的栅格点的值（value）之和
    bestdist = lastbestdist = currentdist;

    do {
	currentpos = lastbestpos;                              //在最后的最佳位置上进行随机偏移
	currentpos.x += 50 * (((double)rand()) / RAND_MAX - 0.5);h      
	currentpos.y += 50 * (((double)rand()) / RAND_MAX - 0.5);
	currentpos.theta += 50 * (((double)rand()) / RAND_MAX - 0.5);

	currentdist = ts_distance_scan_to_map(scan, map, &currentpos); //ts_distance_scan_to_map计算这一帧每个雷达点转化到全局地图中所在的栅格点的值（value）之和
	
	if (currentdist < bestdist) {              //dist越小说明在那些雷达点的地方所对应的地图栅格中有障碍，说明这个位置是越准确的，有障碍TS_OBSTACLE=0
	    bestdist = currentdist;
	    bestpos = currentpos;
            if (debug) printf("Monte carlo ! %lg %lg %lg %d (count = %d)\n", bestpos.x, bestpos.y, bestpos.theta, bestdist, counter);
	} else {
	    counter++;
	}
        if (counter > 100) {
            if (bestdist < lastbestdist) {
                lastbestpos = bestpos;
                lastbestdist = bestdist;
                counter = 0;
            }
        }
    } while (counter < 1000);                //1000个粒子，每个粒子分别定位，找到最匹配的粒子
    return bestpos;
}

typedef struct {
    double r;	    // length wheels' radius                   //车轮半径
    double R;	    // half the wheels' axis length            //轴距一半
    int inc;	    // wheels' counters increments per turn    //车轮每转的计数器增量 
    double ratio;   // ratio between left and right wheel      //左右轮之间的比例
} cart_parameters_t;


typedef struct {
    int timestamp;                                          //相隔时间，(timestamp/100000)s  单位：微妙 
    int q1, q2;                                             //里程计信息，里程计量，程序中q1增加2就相当于转了一圈,应该是q1是左轮，q2是右轮
    ts_scan_t scan;                                         //一整帧的雷达数据，包括雷达点的数目，x,y和 是否有障碍
} ts_sensor_data_t;

ts_sensor_data_t sensor_data[600];                     

int read_sensor_data(ts_sensor_data_t *data)                                                //读雷达数据，返回读入的雷达帧数
{   
    FILE *input;
    int i, j, nb_sensor_data = 0;
    int d[TS_SCAN_SIZE];
    ts_scan_t *scan;
    char *str, line[4000];
    double angle_deg, angle_rad;

    input = fopen(TEST_FILENAME, "rt");
    do {
        // Read the scan
        str = fgets(line, 4000, input);
        if (str == NULL) break;
        str = strtok(str, " ");
        sscanf(str, "%d", &data[nb_sensor_data].timestamp);
        str = strtok(NULL, " ");
        sscanf(str, "%d", &data[nb_sensor_data].q1);
        str = strtok(NULL, " ");
        sscanf(str, "%d", &data[nb_sensor_data].q2);
        data[nb_sensor_data].q2 = -data[nb_sensor_data].q2;
        for (i = 0; i < 10; i++)
            str = strtok(NULL, " ");
        for (i = 0; i < TEST_SCAN_SIZE; i++) {
            if (str) {
                sscanf(str, "%d", &d[i]);                          //d[i],单位：mm
                str = strtok(NULL, " ");   
            } else d[i] = 0;
        }

        // Change to (x,y) scan
        scan = &data[nb_sensor_data].scan;
        scan->nb_points = 0;
#define SPAN 3
        // Span the laser scans to better cover the space
        for (i = 0; i < TEST_SCAN_SIZE; i++) {
            for (j = 0; j != SPAN; j++) {
                angle_deg = TEST_ANGLE_MIN + ((double)(i * SPAN + j)) * (TEST_ANGLE_MAX - TEST_ANGLE_MIN) / (TEST_SCAN_SIZE * SPAN - 1);

                // Correction of angle according to odometry
                //angle_deg += psidotodo_old / 3600.0 * ((double)(i * SPAN + j)) * (TEST_ANGLE_MAX - TEST_ANGLE_MIN) / (TEST_SCAN_SIZE * SPAN - 1);

                angle_rad = angle_deg * M_PI / 180;
                if (i > 45 && i < TEST_SCAN_SIZE - 45) {
                    if (d[i] == 0) {
                        scan->x[scan->nb_points] = TS_DISTANCE_NO_DETECTION * cos(angle_rad);
                        scan->y[scan->nb_points] = TS_DISTANCE_NO_DETECTION * sin(angle_rad);
                        scan->value[scan->nb_points] = TS_NO_OBSTACLE;
                        scan->x[scan->nb_points] += TEST_OFFSET_LASER;
                        scan->nb_points++;
                    }
                    if (d[i] > TEST_HOLE_WIDTH / 2) {
                        scan->x[scan->nb_points] = d[i] * cos(angle_rad);
                        scan->y[scan->nb_points] = d[i] * sin(angle_rad);
                        scan->value[scan->nb_points] = TS_OBSTACLE;                  
                        scan->x[scan->nb_points] += TEST_OFFSET_LASER;
                        scan->nb_points++;
                    }
                }
            }
        }
        nb_sensor_data++;
    } while (1);

    fclose(input);
    return nb_sensor_data;
}

int main()
{
    FILE *output, *output2;
    double angle_rad, angle_deg;
    ts_position_t startpos, position, position2;
    char filename[256];
    int i, x, y, test;
    int nb_sensor_data, cnt_scans;
    int timestamp, told, q1, q2, nq1, nq2;
    double m, v, vodo, thetarad, psidot, thetaradodo, psidotodo, psidotodo_old, vodo_old;
    cart_parameters_t params;

    params.r = 0.077;                          //轮胎半径：7.7cm ，单位：米
    params.R = 0.165;                          //轴距一半：16.5cm，单位：米
    params.inc = 2000;                         ////车轮每转的计数器增量 ，车轮每转计数器增量为2000
    params.ratio = 1.0;                        //左右轮之间的比例

    // Read all the scans
    nb_sensor_data = read_sensor_data(sensor_data);            //返回值nb_sensor_data雷达帧数
    printf("sensor data = %d\n", nb_sensor_data); 

    for (test = 0; test != 1; test++) {
        
        ts_map_init(&map);
        ts_map_init(&trajectory);
        output = fopen("test_trajectory.dat", "wt"); 
        position.x = 0.3 * TS_MAP_SIZE / TS_MAP_SCALE;         //初始位置坐标是0.3*2048/0.1mm  (0.3*2048cm)，在全局地图的0.3分位点处
        position.y = 0.3 * TS_MAP_SIZE / TS_MAP_SCALE; 
        position.theta = 0;
        startpos = position2 = position;                       //position是前一时刻位置，position2是此时刻位置
        //for (cnt_scans = 0; cnt_scans != nb_sensor_data; cnt_scans++) {
        for (cnt_scans = nb_sensor_data - 1; cnt_scans >= 0; cnt_scans--) {               //每一帧分别处理
            timestamp = sensor_data[cnt_scans].timestamp;
            nq1 = sensor_data[cnt_scans].q1;
            nq2 = sensor_data[cnt_scans].q2;

            // Manage robot position
            if (cnt_scans != 0) {
                psidotodo_old = psidotodo;
                vodo_old = vodo;
                m = params.r * M_PI / params.inc;                         
                vodo = m * (nq1 - q1 + (nq2 - q2) * params.ratio);        //机器人相邻两个时刻前进的距离  单位是m
                thetarad = position.theta * M_PI / 180;
                position2 = position;                                     //position:  机器人前一时刻的坐标x,y，算出来结果为mm
                position2.x += vodo * 1000 * cos(thetarad);               //position2：机器人此时刻的坐标x,y ，*1000是把m转化为mm，应该是mm
                position2.y += vodo * 1000 * sin(thetarad);     
                psidotodo = (m * ((nq2 - q2) * params.ratio - nq1 + q1) / params.R) * 180 / M_PI;              //机器人相邻两个时刻偏转角大小
                position2.theta += psidotodo;                             //机器人此时刻的偏转角theta
                vodo *= 1000000.0 / (timestamp - told);                   //机器人的线速度，时间是 (timestamp/100000)s ，单位是m/s
                psidotodo *= 1000000.0 / (timestamp - told);              //机器人的角速度      
                fprintf(output, "%d %d %lg %lg %lg %lg %lg", cnt_scans, timestamp - told, position2.x, position2.y, position2.theta, fabs(vodo), psidotodo);
            } else {
                psidotodo_old = psidotodo = 0;
                vodo_old = vodo = 0;
            }

            position2 = monte_carlo_move(&sensor_data[cnt_scans].scan, &map, &position, 0); //上一时刻位置周围进行蒙特卡罗移动，找到最优的，作为此时刻的位置，就没有用到里程计信息
            v = sqrt((position2.x - position.x) * (position2.x - position.x) +
                    (position2.y - position.y) * (position2.y - position.y));              //v：相邻时刻位置的直线距离          
            psidot = position2.theta - position.theta;                                     //psidot：机器人相邻两个时刻偏转角大小
            if (cnt_scans != 0) {
                v *= 1000.0 / (timestamp - told);
                psidot *= 1000000.0 / (timestamp - told); 
            } else {
                v = 0;
                psidot = 0;
            }
            if (cnt_scans) {
                fprintf(output, " %lg %lg", v, psidot); 
            }

            position = position2;                                                          //此时，position是此时刻机器人位置                    
            printf("#%d : %lg %lg %lg\n", cnt_scans, position.x, position.y, position.theta);
            ts_map_update(&sensor_data[cnt_scans].scan, &map, &position, 50, TEST_HOLE_WIDTH);

            x = (int)floor(position.x * TS_MAP_SCALE + 0.5);
            y = ((int)floor(position.y * TS_MAP_SCALE + 0.5));
            if (x >= 0 && y >= 0 && x < TS_MAP_SIZE && y < TS_MAP_SIZE)
                trajectory.map[y * TS_MAP_SIZE + x] = 0;                              //机器人走过的路径位置值置为0

            if (0) { 
                sprintf(filename, "test_lab_dist%04d.txt", cnt_scans);
                output2 = fopen(filename, "wt");
                position2 = position;
                for (i = 0; i < 500; i++) {
                    position2.x++;
                    fprintf(output2, "%d\n", ts_distance_scan_to_map(&sensor_data[cnt_scans].scan, &map, &position2));
                }
                fclose(output2);
            }
            //if (nbscans > 310 && nbscans < 350)
            {
                //ts_map_init(&trajectory);
                draw_scan(&sensor_data[cnt_scans].scan, &trajectory, &position); //雷达扫描点转化到trajectory地图中，
                sprintf(filename, "test_lab%04d.pgm", cnt_scans);
                //record_map(state.map, &trajectory, filename, 1600, 1200);
            }
            fprintf(output, "\n");
            q1 = nq1;
            q2 = nq2;
            told = timestamp;
        }
        fclose(output);

        // Record the map
        sprintf(filename, "test_lab_reverse%04d.pgm", test);
        record_map(&map, &trajectory, filename, TS_MAP_SIZE, TS_MAP_SIZE);
    }
    return 1;
}
