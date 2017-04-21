// All dimensions in mm.


$fn = 360;

// difference() fudge factor
fuj = 0.2;





//
// Modules
//

module clamp(n=1) {

  clamp_basic();

}


module clamp_basic(clamp_l=10) {

  clamp_w=5;
  notch_h = 3;
  notch_z = 5;  // distance from bottom of body to center of notch

  clamp_h = notch_z + (notch_h/2) + 1;

  difference() {
    // body
    cube([clamp_w, clamp_l, clamp_h]);
    // notch
#   translate([clamp_w, clamp_l/2, notch_z])
      rotate([0, 45, 0])
        cube([notch_h/1.41, clamp_l+fuj, notch_h/1.41], center=true);
  }
}





//
// Main
//

X = 2;
Y = 4;
Z = 6;

clamp();
