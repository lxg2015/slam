#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#ifdef _MSC_VER
   typedef __int64 int64_t;	// Define it from MSVC's internal type
#else
   #include <stdint.h>		// Use the C99 official header
#endif
#include "CoreSLAM.h"

void 
ts_map_init(ts_map_t *map)                                //初始化地图把地图每个栅格的值（value）都置为(TS_OBSTACLE + TS_NO_OBSTACLE) / 2；介于障碍和无障碍之间(未知)
{
    int x, y, initval;
    ts_map_pixel_t *ptr;
    initval = (TS_OBSTACLE + TS_NO_OBSTACLE) / 2;
    for (ptr = map->map, y = 0; y < TS_MAP_SIZE; y++) {
	for (x = 0; x < TS_MAP_SIZE; x++, ptr++) {
	    *ptr = initval;
	}
    }
}

int
ts_distance_scan_to_map(ts_scan_t *scan, ts_map_t *map, ts_position_t *pos)               //把机器人坐标系下的障碍点映射到全局坐标系地图下，计算
{
    double c, s;
    int i, x, y, nb_points = 0;
    int64_t sum;

    c = cos(pos->theta * M_PI / 180);
    s = sin(pos->theta * M_PI / 180);
    // Translate and rotate scan to robot position
    // and compute the distance
    for (i = 0, sum = 0; i != scan->nb_points; i++) {
        if (scan->value[i] != TS_NO_OBSTACLE) {
            x = (int)floor((pos->x + c * scan->x[i] - s * scan->y[i]) * TS_MAP_SCALE + 0.5);    //x，y是雷达点在全局地图中所在的的栅格行数列数
            y = (int)floor((pos->y + s * scan->x[i] + c * scan->y[i]) * TS_MAP_SCALE + 0.5);
            // Check boundaries
            if (x >= 0 && x < TS_MAP_SIZE && y >= 0 && y < TS_MAP_SIZE) {
                sum += map->map[y * TS_MAP_SIZE + x];                           //sum计算这一帧每个雷达点转化到全局地图中所在的栅格点的值（value）之和
                nb_points++;
            }
        }
    }
    if (nb_points) sum = sum * 1024 / nb_points;
    else sum = 2000000000;
    return (int)sum;
}

#define SWAP(x, y) (x ^= y ^= x ^= y)

void
ts_map_laser_ray(ts_map_t *map, int x1, int y1, int x2, int y2, 
                 int xp, int yp, int value, int alpha)
																	// x1, y1: 机器人在全局地图的位置，单位:cm
																	// x2, y2: 某一个雷达点所在的“圆洞”（holes）的最远端在全局地图的位置，单位：cm
																	// xp, yp: 某一个雷达点在全局地图的位置，单位：cm 
																	// alpha: TS_NO_OBSTACLE时， q = 50 / 4; TS_OBSTACLE。
																	// 如果该雷达点处值（栅格值）为TS_NO_OBSTACLE,value则为TS_NO_OBSTACLE，65500
																	// 如果其他情况,value则为TS_NO_OBSTACLE

{
    int x2c, y2c, dx, dy, dxc, dyc, error, errorv, derrorv, x;
    int incv, sincv, incerrorv, incptrx, incptry, pixval, horiz, diago;
    ts_map_pixel_t *ptr;

    if (x1 < 0 || x1 >= TS_MAP_SIZE || y1 < 0 || y1 >= TS_MAP_SIZE)
        return; // Robot is out of map
    
    x2c = x2; y2c = y2;
    // Clipping                                                                 
	// 如果雷达点的位置超出地图范围，就取雷达点所在的那条线与地图的交点处为（x2c，y2c）；
	// 否则雷达点位置未超出地图范围，就取真实雷达点坐标为（x2c，y2c）；
    if (x2c < 0) {
        if (x2c == x1) return;
        y2c += (y2c - y1) * (-x2c) / (x2c - x1);
        x2c = 0;
    }
    if (x2c >= TS_MAP_SIZE) {
        if (x1 == x2c) return;
        y2c += (y2c - y1) * (TS_MAP_SIZE - 1 - x2c) / (x2c - x1);
        x2c = TS_MAP_SIZE - 1;
    }
    if (y2c < 0) {
        if (y1 == y2c) return;
        x2c += (x1 - x2c) * (-y2c) / (y1 - y2c);
        y2c = 0;
    }
    if (y2c >= TS_MAP_SIZE) {
        if (y1 == y2c) return;
        x2c += (x1 - x2c) * (TS_MAP_SIZE - 1 - y2c) / (y1 - y2c);
        y2c = TS_MAP_SIZE - 1;
    }
		// x1, y1: 机器人在全局地图的位置，单位:cm
		// x2, y2: 某一个雷达点所在的“圆洞”（holes）的最远端在全局地图的理论位置，单位：cm
		// xp, yp: 某一个雷达点在全局地图的位置，单位：cm 
		// x2c，y2c : 某一个雷达点所在的“圆洞”（holes）的最远端在全局地图中的实际位置（可能超出地图范围），单位:cm，
		// value:  TS_NO_OBSTACLE 或者TS_OBSTACLE
    dx = abs(x2 - x1); dy = abs(y2 - y1);                                 //dx:  雷达点“圆洞”理论位置和机器人位置的差值
    dxc = abs(x2c - x1); dyc = abs(y2c - y1);                             //dxc: 雷达点“圆洞”实际位置和机器人位置的差值
    incptrx = (x2 > x1) ? 1 : -1;                                         
    incptry = (y2 > y1) ? TS_MAP_SIZE : -TS_MAP_SIZE;
    sincv = (value > TS_NO_OBSTACLE) ? 1 : -1; 
    if (dx > dy) {
        derrorv = abs(xp - x2);                                           //derrorv: 雷达点“圆洞”理论位置和雷达点位置的差值
    } else {
        SWAP(dx, dy); SWAP(dxc, dyc); SWAP(incptrx, incptry);        
        derrorv = abs(yp - y2);
    }
    error = 2 * dyc - dxc;
    horiz = 2 * dyc;
    diago = 2 * (dyc - dxc);
    errorv = derrorv / 2;
    incv = (value - TS_NO_OBSTACLE) / derrorv;                            //incv非正，或者为负，或者为0
    incerrorv = value - TS_NO_OBSTACLE - derrorv * incv;  
    ptr = map->map + y1 * TS_MAP_SIZE + x1;                               // ptr指针指向机器人位置（x1,y1）
    pixval = TS_NO_OBSTACLE;                                              // pixval 初始值为 TS_NO_OBSTACLE，无障碍，65500
    for (x = 0; x <= dxc; x++, ptr += incptrx)                            // ptr依次指向下一个位置，从机器人位置到雷达点“圆洞”实际位置
	{
        if (x > dx - 2 * derrorv) 
		{
            if (x <= dx - derrorv)       //s1                                    //dx - derrorv = abs(x2 - x1)- abs(xp - x2)=abs(xp - x1)    
			{
                pixval += incv;                                            //incv 必定为负，机器人和雷达点之间的点pixval 越来越小，越有障碍
                //如果雷达点所在栅格越可能“有”障碍，雷达点的value值越小，则incv是负的，且incv绝对值越大，则pixval越小，则机器人到雷达点之间的栅格点越可能“有”障碍
                errorv += incerrorv;
                if (errorv > derrorv) 
				{
                    pixval += sincv;                                       //主要修改pixval和errorv
                    errorv -= derrorv; 
			    }
            }
			else                       //s2
			{
                pixval -= incv;                                           //雷达点后面的点 pixval 越来越大，
				 //如果雷达点所在栅格越可能“有”障碍，雷达点的value值越小，则incv是负的，且incv绝对值越大，则pixval越大，则雷达点到“圆洞”之间的栅格点越可能”没有“障碍
                errorv -= incerrorv;
                if (errorv < 0) 
				{
                    pixval -= sincv;
                    errorv += derrorv; 
                }
            }
        } 
        // Integration into the map
        *ptr = ((256 - alpha) * (*ptr) + alpha * pixval) >> 8;  	     //栅格赋值
		//*ptr = ( 256*(*ptr)+  alpha*(pixval- (*ptr)) ) >> 8;   如果pixval- (*ptr)=0；则*ptr值不变；即如果求得的栅格值和，
		                                                                 //pixval和*ptr差的越大，则变化值越大，结果越偏向pixval
		//如果雷达点value是TS_NO_OBSTACLE(65500)，则pixval差不多保持不变，恒为TS_NO_OBSTACLE。所有的*ptr越偏向TS_NO_OBSTACLE
		//如果雷达点value是TS_OBSTACLE(0)，pixval在机器人位置为TS_NO_OBSTACLE。pixval在机器人到雷达点位置这一段越来越小（线性减小），在雷达点到”圆洞“最远处越来越大（线性增大）。
		//所有的*ptr越偏向TS_NO_OBSTACLE
        if (error > 0) 
		{
            ptr += incptry;
            error += diago;
        } else error += horiz;
    }
}

void
ts_map_update(ts_scan_t *scan, ts_map_t *map, ts_position_t *pos, int quality, int hole_width)                 //quality =50,  hole_width = 600
{
    double c, s;
    double x2p, y2p;
    int i, x1, y1, x2, y2, xp, yp, value, q;
    double add, dist;

    c = cos(pos->theta * M_PI / 180);
    s = sin(pos->theta * M_PI / 180);
    x1 = (int)floor(pos->x * TS_MAP_SCALE + 0.5);                       //机器人在全局地图的位置，单位:cm
    y1 = (int)floor(pos->y * TS_MAP_SCALE + 0.5);
    // Translate and rotate scan to robot position
    for (i = 0; i != scan->nb_points; i++) {
        x2p = c * scan->x[i] - s * scan->y[i];                       //某一个雷达点相对于机器人位置在全局坐标下的x方向距离偏差和y方向距离偏差，单位：mm
        y2p = s * scan->x[i] + c * scan->y[i];                     
        xp = (int)floor((pos->x + x2p) * TS_MAP_SCALE + 0.5);        //某一个雷达点在全局坐标下的位置， *TS_MAP_SCALE，mm转化为cm，单位：cm 
        yp = (int)floor((pos->y + y2p) * TS_MAP_SCALE + 0.5);
        dist = sqrt(x2p * x2p + y2p * y2p);                          //dist  :某一个雷达点与机器人位置的距离，单位: mm               
        add = hole_width / 2 / dist;                                    //hole_width = 600
        x2p *= TS_MAP_SCALE * (1 + add);
        y2p *= TS_MAP_SCALE * (1 + add); 
        x2 = (int)floor(pos->x * TS_MAP_SCALE + x2p + 0.5);          
        y2 = (int)floor(pos->y * TS_MAP_SCALE + y2p + 0.5);
        if (scan->value[i] == TS_NO_OBSTACLE) {            
            q = quality / 4;
            value = TS_NO_OBSTACLE;
        } else {
            q = quality;
            value = TS_OBSTACLE;
        }
        //printf("%d %d %d %d %d %d %d\n", i, x1, y1, x2, y2, xp, yp);
        ts_map_laser_ray(map, x1, y1, x2, y2, xp, yp, value, q);    // x1, y1: 机器人在全局地图的位置，单位:cm
																	// x2, y2: 某一个雷达点所在的“圆洞”（holes）的最远端在全局地图的位置，单位：cm
																	// xp, yp: 某一个雷达点在全局地图的位置，单位：cm 
																	// q: TS_NO_OBSTACLE时， q = quality / 4; TS_OBSTACLE时， q = quality;
																	//value: TS_NO_OBSTACLE 或者TS_OBSTACLE
    }
}

