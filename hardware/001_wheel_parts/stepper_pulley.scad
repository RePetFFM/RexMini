$fn = 32;
teeth = 22; // 64,68mm
motor_shaft = 1;
pulley_t_ht = 10;

include <Pulley_T-MXL-XL-HTD-GT2_N-tooth.scad>;

shaft_radius = 4.4/2;


difference() {
	union() {
		pulley ( "T2.5" , T2_5_pulley_dia , 0.7 , 1.678);
		hull() {
			cylinder(r=15/2,h=3.5);
			cylinder(r=18/2,h=3);
		}
	}

	difference() {
		translate([0,0,-1]) cylinder(r=shaft_radius,h=20);
		translate([5+1.7,0,0]) cube([10,10,10],center=true);	
	}	
}	



