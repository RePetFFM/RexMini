$fn = 12;
teeth = 40; // 64,68mm
motor_shaft = 58;
pulley_t_ht = 9+4;

include <Pulley_T-MXL-XL-HTD-GT2_N-tooth.scad>

import("wheel_pulley_base.stl");


union() {
	pulley ( "XL" , XL_pulley_dia , 1.27, 3.051 );

	
	difference() {
	    
		hull() {
			cylinder(r=32.2,h=4,$fn=64);
			cylinder(r=33,h=3,$fn=64);	
		}
		translate([0,0,-1]) cylinder(r=(motor_shaft/2)+0.2,h=6,$fn=64);
	}

}