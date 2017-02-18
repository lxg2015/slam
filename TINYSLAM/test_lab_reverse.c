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


//��¼��ͼ����¼ȫ�ֵ�ͼ���ѻ������߹�·���ϵ�դ����Ϊ0 �����ϰ�Ϊ0
void record_map(ts_map_t *map, ts_map_t *overlay, char *filename, int width, int height)  //width��height = TS_MAP_SIZE
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
		    if (overlay->map[ (TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x] == 0)    //���overlay·����ͼ�л������߹����դ�������դ��ֵ���0
                fprintf(output, "0 ");
            else                                                                //�������ȫ�ֵ�ͼmap�е�ֵ
                fprintf(output, "%d ", (int)(map->map[ (TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x]) >> 8);
		}
		fprintf(output, "\n");
    }
    fclose(output);
}

void draw_scan(ts_scan_t *scan, ts_map_t *map, ts_position_t *pos)    //�ѻ�����ɨ����״��ת����ȫ�ֵ�ͼ�У�դ����Ϊ0��TS_OBSTACLE
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
//scan: ��һ֡���״�����;    map: ȫ�ֵ�ͼ��  start_pos:��һʱ�̵Ļ�����λ�ã�   debug:
{
    ts_position_t cpp, currentpos, bestpos, lastbestpos;
    int currentdist;
    int bestdist, lastbestdist;
    int counter = 0;

    currentpos = bestpos = lastbestpos = *start_pos;
    currentdist = ts_distance_scan_to_map(scan, map, &currentpos);    //sum������һ֡ÿ���״��ת����ȫ�ֵ�ͼ�����ڵ�դ����ֵ��value��֮��
    bestdist = lastbestdist = currentdist;

    do {
	currentpos = lastbestpos;                              //���������λ���Ͻ������ƫ��
	currentpos.x += 50 * (((double)rand()) / RAND_MAX - 0.5);h      
	currentpos.y += 50 * (((double)rand()) / RAND_MAX - 0.5);
	currentpos.theta += 50 * (((double)rand()) / RAND_MAX - 0.5);

	currentdist = ts_distance_scan_to_map(scan, map, &currentpos); //ts_distance_scan_to_map������һ֡ÿ���״��ת����ȫ�ֵ�ͼ�����ڵ�դ����ֵ��value��֮��
	
	if (currentdist < bestdist) {              //distԽС˵������Щ�״��ĵط�����Ӧ�ĵ�ͼդ�������ϰ���˵�����λ����Խ׼ȷ�ģ����ϰ�TS_OBSTACLE=0
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
    } while (counter < 1000);                //1000�����ӣ�ÿ�����ӷֱ�λ���ҵ���ƥ�������
    return bestpos;
}

typedef struct {
    double r;	    // length wheels' radius                   //���ְ뾶
    double R;	    // half the wheels' axis length            //���һ��
    int inc;	    // wheels' counters increments per turn    //����ÿת�ļ��������� 
    double ratio;   // ratio between left and right wheel      //������֮��ı���
} cart_parameters_t;


typedef struct {
    int timestamp;                                          //���ʱ�䣬(timestamp/100000)s  ��λ��΢�� 
    int q1, q2;                                             //��̼���Ϣ����̼�����������q1����2���൱��ת��һȦ,Ӧ����q1�����֣�q2������
    ts_scan_t scan;                                         //һ��֡���״����ݣ������״�����Ŀ��x,y�� �Ƿ����ϰ�
} ts_sensor_data_t;

ts_sensor_data_t sensor_data[600];                     

int read_sensor_data(ts_sensor_data_t *data)                                                //���״����ݣ����ض�����״�֡��
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
                sscanf(str, "%d", &d[i]);                          //d[i],��λ��mm
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

    params.r = 0.077;                          //��̥�뾶��7.7cm ����λ����
    params.R = 0.165;                          //���һ�룺16.5cm����λ����
    params.inc = 2000;                         ////����ÿת�ļ��������� ������ÿת����������Ϊ2000
    params.ratio = 1.0;                        //������֮��ı���

    // Read all the scans
    nb_sensor_data = read_sensor_data(sensor_data);            //����ֵnb_sensor_data�״�֡��
    printf("sensor data = %d\n", nb_sensor_data); 

    for (test = 0; test != 1; test++) {
        
        ts_map_init(&map);
        ts_map_init(&trajectory);
        output = fopen("test_trajectory.dat", "wt"); 
        position.x = 0.3 * TS_MAP_SIZE / TS_MAP_SCALE;         //��ʼλ��������0.3*2048/0.1mm  (0.3*2048cm)����ȫ�ֵ�ͼ��0.3��λ�㴦
        position.y = 0.3 * TS_MAP_SIZE / TS_MAP_SCALE; 
        position.theta = 0;
        startpos = position2 = position;                       //position��ǰһʱ��λ�ã�position2�Ǵ�ʱ��λ��
        //for (cnt_scans = 0; cnt_scans != nb_sensor_data; cnt_scans++) {
        for (cnt_scans = nb_sensor_data - 1; cnt_scans >= 0; cnt_scans--) {               //ÿһ֡�ֱ���
            timestamp = sensor_data[cnt_scans].timestamp;
            nq1 = sensor_data[cnt_scans].q1;
            nq2 = sensor_data[cnt_scans].q2;

            // Manage robot position
            if (cnt_scans != 0) {
                psidotodo_old = psidotodo;
                vodo_old = vodo;
                m = params.r * M_PI / params.inc;                         
                vodo = m * (nq1 - q1 + (nq2 - q2) * params.ratio);        //��������������ʱ��ǰ���ľ���  ��λ��m
                thetarad = position.theta * M_PI / 180;
                position2 = position;                                     //position:  ������ǰһʱ�̵�����x,y����������Ϊmm
                position2.x += vodo * 1000 * cos(thetarad);               //position2�������˴�ʱ�̵�����x,y ��*1000�ǰ�mת��Ϊmm��Ӧ����mm
                position2.y += vodo * 1000 * sin(thetarad);     
                psidotodo = (m * ((nq2 - q2) * params.ratio - nq1 + q1) / params.R) * 180 / M_PI;              //��������������ʱ��ƫת�Ǵ�С
                position2.theta += psidotodo;                             //�����˴�ʱ�̵�ƫת��theta
                vodo *= 1000000.0 / (timestamp - told);                   //�����˵����ٶȣ�ʱ���� (timestamp/100000)s ����λ��m/s
                psidotodo *= 1000000.0 / (timestamp - told);              //�����˵Ľ��ٶ�      
                fprintf(output, "%d %d %lg %lg %lg %lg %lg", cnt_scans, timestamp - told, position2.x, position2.y, position2.theta, fabs(vodo), psidotodo);
            } else {
                psidotodo_old = psidotodo = 0;
                vodo_old = vodo = 0;
            }

            position2 = monte_carlo_move(&sensor_data[cnt_scans].scan, &map, &position, 0); //��һʱ��λ����Χ�������ؿ����ƶ����ҵ����ŵģ���Ϊ��ʱ�̵�λ�ã���û���õ���̼���Ϣ
            v = sqrt((position2.x - position.x) * (position2.x - position.x) +
                    (position2.y - position.y) * (position2.y - position.y));              //v������ʱ��λ�õ�ֱ�߾���          
            psidot = position2.theta - position.theta;                                     //psidot����������������ʱ��ƫת�Ǵ�С
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

            position = position2;                                                          //��ʱ��position�Ǵ�ʱ�̻�����λ��                    
            printf("#%d : %lg %lg %lg\n", cnt_scans, position.x, position.y, position.theta);
            ts_map_update(&sensor_data[cnt_scans].scan, &map, &position, 50, TEST_HOLE_WIDTH);

            x = (int)floor(position.x * TS_MAP_SCALE + 0.5);
            y = ((int)floor(position.y * TS_MAP_SCALE + 0.5));
            if (x >= 0 && y >= 0 && x < TS_MAP_SIZE && y < TS_MAP_SIZE)
                trajectory.map[y * TS_MAP_SIZE + x] = 0;                              //�������߹���·��λ��ֵ��Ϊ0

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
                draw_scan(&sensor_data[cnt_scans].scan, &trajectory, &position); //�״�ɨ���ת����trajectory��ͼ�У�
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
