// OnSpeed ESP32-S3 board enclosure, for 3D printing (JLC3DP: MJF PA12 nylon recommended).
// Board coordinates come from the KiCad layout: board x = X, board y (front 0 .. rear 80) maps to
// Y = 80 - y, so the rear connectors face -Y. Z = 0 is the outside of the base.
// Two parts: base (part="base") and lid (part="lid"). Render both with part="both".
//
// Values marked CHECK come from drawings that were not fully legible; measure the first
// assembled board before printing in quantity.

part = "both";
$fn = 48;

W = 100; L = 80;              // board
wall = 2.5; floor_t = 2.4; lid_t = 2.4;
gap_side = 0.6;               // board to side walls
gap_rear = 0.5;               // D-sub flange sits against the rear wall
gap_front = 16;               // room for the pitot/AOA tubes to bend to the front fittings
standoff = 5.0;               // board bottom above the floor (THT leads, bottom test pads)
pcb_t = 1.6;
inner_h = standoff + pcb_t + 16;   // tallest part: D-sub housing about 12.6 mm (CHECK)

board_z = floor_t + standoff;      // board bottom
top_z = board_z + pcb_t;           // board top surface
ix0 = -gap_side; ix1 = W + gap_side;
iy0 = -gap_rear; iy1 = L + gap_front;
ox0 = ix0 - wall; ox1 = ix1 + wall; oy0 = iy0 - wall; oy1 = iy1 + wall;
H = floor_t + inner_h;
r_out = 3;

holes = [[4.5, 27.5], [4.5, 59.0], [95.5, 4.5], [95.8, 47.0]];   // KiCad x, y
function B(p) = [p[0], L - p[1]];

// rear connectors (CHECK centre heights against the CONNFLY and HRO drawings)
dsub_cz = top_z + 6.3;
db15_x = 77.305; de9_x = 18.54; usb_x = 45.0; usb_cz = top_z + 1.63;

module rrect(w, h, r) { offset(r) square([w - 2 * r, h - 2 * r], center = true); }
module shell2d(x0, y0, x1, y1, r) { translate([(x0 + x1) / 2, (y0 + y1) / 2]) rrect(x1 - x0, y1 - y0, r); }

module rear_cutouts() {
    // cut through the rear wall (y from oy0-1 to iy0+1)
    for (c = [[db15_x, 27.6, 33.32], [de9_x, 19.4, 24.99]]) {
        translate([c[0], iy0 + 1, dsub_cz]) rotate([90, 0, 0]) linear_extrude(wall + 2)
            hull() {                                     // D-shaped opening, wide side up
                translate([-c[1] / 2 + 1, 4.6]) circle(1);
                translate([c[1] / 2 - 1, 4.6]) circle(1);
                translate([-c[1] / 2 + 2.4, -4.6]) circle(1);
                translate([c[1] / 2 - 2.4, -4.6]) circle(1);
            }
        for (s = [-1, 1]) translate([c[0] + s * c[2] / 2, iy0 + 1, dsub_cz]) rotate([90, 0, 0]) cylinder(d = 3.4, h = wall + 2);
    }
    translate([usb_x, iy0 + 1, usb_cz]) rotate([90, 0, 0]) linear_extrude(wall + 2) rrect(10.2, 4.6, 1.6);
}

module front_cutouts() {
    // three bulkhead air fittings at the Gen2 front-panel spacing (0.70 in), centred on the board
    fit = [[W / 2 - 17.8, 0], [W / 2, 6.6], [W / 2 + 17.8, 0]];
    for (f = fit) translate([f[0], iy1 - 1, top_z + 9 + f[1]]) rotate([-90, 0, 0]) cylinder(d = 7.1, h = wall + 2);
    // labels: P-fwd, P45, static
}

module base() {
    difference() {
        union() {
            difference() {
                linear_extrude(H) shell2d(ox0, oy0, ox1, oy1, r_out);
                translate([0, 0, floor_t]) linear_extrude(H) shell2d(ix0, iy0, ix1, iy1, 1);
            }
            // board standoffs
            for (h = holes) translate([B(h)[0], B(h)[1], 0]) cylinder(d = 7, h = board_z);
            // lid screw posts in the inside corners
            for (p = [[ix0, iy0], [ix1, iy0], [ix0, iy1], [ix1, iy1]])
                translate([p[0], p[1], 0]) cylinder(d = 8, h = H);
            // mounting ears
            for (sx = [ox0 - 9, ox1]) translate([sx, oy0 + 12, 0]) cube([9, oy1 - oy0 - 24, 4]);
        }
        rear_cutouts();
        front_cutouts();
        for (h = holes) translate([B(h)[0], B(h)[1], floor_t]) cylinder(d = 4.0, h = 20);     // M3 heat-set insert
        for (p = [[ix0, iy0], [ix1, iy0], [ix0, iy1], [ix1, iy1]])
            translate([p[0], p[1], H - 8]) cylinder(d = 4.0, h = 9);                         // M3 insert for the lid
        for (sx = [ox0 - 4.5, ox1 + 4.5]) for (yy = [oy0 + 18, oy1 - 18])
            translate([sx, yy, -1]) cylinder(d = 4.4, h = 6);                               // airframe mounting, #8 / M4
        // keep the corner posts out of the board's path
        translate([ix0 + 0.01, iy0, board_z - 0.5]) cube([W + 2 * gap_side - 0.02, L + 0.5, pcb_t + 1]);
    }
}

module lid() {
    led = [[20.4 + 3.2, 29.4], [20.4 + 3.2, 26.6]];   // status (blue) and power (green) LEDs
    difference() {
        linear_extrude(lid_t) shell2d(ox0, oy0, ox1, oy1, r_out);
        for (p = [[ix0, iy0], [ix1, iy0], [ix0, iy1], [ix1, iy1]]) translate([p[0], p[1], -1]) {
            cylinder(d = 3.4, h = lid_t + 2);
            translate([0, 0, lid_t - 1.6]) cylinder(d = 6.2, h = 2);                       // counterbore
        }
        for (l = led) translate([B(l)[0], B(l)[1], -1]) cylinder(d = 3.0, h = lid_t + 2);   // light-pipe holes
        translate([(ox0 + ox1) / 2, (oy0 + oy1) / 2, lid_t - 0.4]) linear_extrude(1)
            text("ONSPEED", size = 9, halign = "center", valign = "center", font = "Liberation Sans:style=Bold");
    }
}

if (part == "base") base();
if (part == "lid") lid();
if (part == "both") { base(); translate([0, 0, H + 10]) lid(); }
