#include <stdio.h>
#include <proj.h>
#include <geodesic.h>

PJ* pjt_init(double lat0, double lon0) {
    PJ_CONTEXT *C = PJ_DEFAULT_CTX;
    char buf[512];
    sprintf(buf, "+proj=etmerc +lat_0=%lf +lon_0=%lf +ellps=WGS84 +units=m +no_defs +k_0=0.999966667 +x0=0 +y_0=0", lat0, lon0);
    PJ* P = proj_create (C, buf);
    return P;
}

// Global coordinate to local coordinate
int lla_to_enu(PJ* P, double* lla, double* enu) {
    PJ_COORD a = proj_coord (proj_torad(lla[1]), proj_torad(lla[0]), lla[2], 0);
    PJ_COORD b = proj_trans (P, PJ_FWD, a);
    enu[0] = b.v[0]; // b.enu.e
    enu[1] = b.v[1]; // b.enu.n
    enu[2] = b.v[2]; // // b.enu.n
    return 0;
}

// Local coordinate to global coordinate
int enu_to_lla(PJ* P, double* enu, double* lla) {
    PJ_COORD a = proj_coord (enu[0], enu[1], enu[2], 0);
    PJ_COORD b = proj_trans (P, PJ_INV, a);
    lla[0] = proj_todeg(b.v[1]);
    lla[1] = proj_todeg(b.v[0]);
    lla[2] = b.v[2];
    return 0;
}

// Geodesic length (in meters) between two (lat, lon) pairs
double geodesic_lla(PJ* P, double* lla1, double* lla2) {
    PJ_COORD a = proj_coord (proj_torad(lla1[1]), proj_torad(lla1[0]), lla1[2], 0);
    PJ_COORD b = proj_coord (proj_torad(lla2[1]), proj_torad(lla2[0]), lla2[2], 0);
    PJ_COORD c = proj_geod(P, a, b);
    return c.v[0]; // c.geod.s
}

int pjt_exit(PJ* P) {
    /* Clean up */
    proj_destroy (P);
    return 0;
}
