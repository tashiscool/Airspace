package org.tash.extensions.reservation;

import org.tash.data.GeoCoordinate;

import java.util.HashMap;
import java.util.Map;

/**
 * Small built-in catalog for legacy CARF regression fixtures. Larger DAFIF or
 * NASR ingestion should plug in through {@link CarfWaypointResolver}.
 */
public class DefaultCarfWaypointResolver extends MapWaypointResolver {
    public DefaultCarfWaypointResolver() {
        super(defaultWaypoints());
    }

    private static Map<String, GeoCoordinate> defaultWaypoints() {
        Map<String, GeoCoordinate> points = new HashMap<>();
        put(points, "RAVEC", 30.20, -86.53);
        put(points, "CEW", 30.78, -86.52);
        put(points, "IAH", 29.98, -95.34);
        put(points, "JCT", 30.60, -99.82);
        put(points, "ELP", 31.80, -106.28);
        put(points, "CRG", 30.34, -81.51);
        put(points, "PXT", 38.29, -76.40);
        put(points, "RAV", 41.09, -77.99);
        put(points, "ZER", 40.78, -76.37);
        put(points, "GNF", 33.83, -89.80);
        put(points, "CZT", 28.52, -99.82);
        put(points, "SGJ", 29.96, -81.34);
        put(points, "LFT", 30.20, -91.99);
        put(points, "BSR", 36.18, -121.64);
        put(points, "EYW", 24.58, -81.80);
        put(points, "LAS", 36.08, -115.15);
        put(points, "IMM", 26.43, -81.40);
        put(points, "ISO", 35.33, -77.62);
        put(points, "FHB", 30.61, -81.46);
        put(points, "LFI", 37.08, -76.36);
        put(points, "GSB", 35.34, -77.96);
        put(points, "PIE", 27.91, -82.69);
        put(points, "POC", 34.09, -117.78);
        put(points, "POM", 34.08, -117.75);
        put(points, "TFD", 32.88, -111.91);
        put(points, "IPL", 32.75, -115.50);
        put(points, "BZA", 32.77, -114.60);
        put(points, "MOHAK", 32.62, -113.98);
        put(points, "GBN", 32.96, -112.67);
        put(points, "SSO", 32.27, -109.26);
        put(points, "EWM", 31.95, -106.27);
        put(points, "FST", 30.95, -102.98);
        put(points, "THX", 30.50, -97.97);
        put(points, "NQI", 27.51, -97.81);
        put(points, "EKR", 40.07, -107.92);
        put(points, "SNY", 41.10, -102.98);
        put(points, "OBH", 41.38, -98.35);
        put(points, "DSM", 41.44, -93.65);
        put(points, "IOW", 41.52, -91.61);
        put(points, "JOT", 41.55, -88.32);
        put(points, "GSH", 41.53, -86.03);
        put(points, "DJB", 41.36, -82.16);
        put(points, "ALB", 42.75, -73.80);
        put(points, "BUF", 42.93, -78.65);
        put(points, "YXU", 43.03, -81.15);
        put(points, "ECK", 43.26, -82.72);
        put(points, "PECOK", 43.55, -85.35);
        put(points, "GRB", 44.56, -88.20);
        put(points, "BAE", 43.12, -88.28);
        put(points, "DBQ", 42.40, -90.71);
        put(points, "PWE", 40.20, -95.59);
        put(points, "HLC", 39.38, -99.83);
        put(points, "PUB", 38.29, -104.43);
        put(points, "RSK", 36.75, -108.10);
        put(points, "YHZ", 44.88, -63.51);
        put(points, "YQI", 43.83, -66.09);
        put(points, "ACK", 41.25, -70.06);
        put(points, "RIFLE", 39.52, -78.77);
        put(points, "SWL", 37.97, -75.47);
        put(points, "SAWED", 37.36, -76.01);
        put(points, "ORF", 36.89, -76.20);
        put(points, "WEAVR", 35.65, -77.03);
        put(points, "ISO", 35.34, -77.61);
        put(points, "JMACK", 34.35, -79.08);
        put(points, "SSC", 33.97, -80.47);
        put(points, "CHS", 32.89, -80.04);
        put(points, "TWINS", 32.55, -82.30);
        put(points, "MCN", 32.69, -83.65);
        put(points, "WRB", 32.64, -83.59);
        put(points, "DOVEY", 42.03, -67.00);
        put(points, "LFV", 41.28, -70.00);
        put(points, "AWK", 19.28, 166.65);
        put(points, "CANOE", 24.50, 153.00);
        put(points, "MLT", 25.25, 151.25);
        put(points, "AGIKA", 30.40, 132.40);
        put(points, "SEPIA", 31.00, 131.50);
        put(points, "BOMAP", 31.30, 130.80);
        put(points, "RUSAR", 32.20, 129.30);
        put(points, "ESBIS", 33.00, 128.40);
        put(points, "RUGMA", 33.70, 127.60);
        put(points, "CJU", 33.51, 126.49);
        put(points, "IPDAS", 34.00, 125.70);
        put(points, "KWA", 35.12, 126.81);
        put(points, "LINTA", 35.50, 127.40);
        put(points, "RINBO", 35.80, 128.00);
        put(points, "ENTEL", 36.10, 128.60);
        put(points, "OLMEN", 36.40, 129.20);
        put(points, "SOT", 36.70, 129.80);
        put(points, "LEEEE", 30.40, -113.80);
        put(points, "BTY", 36.80, -116.75);
        put(points, "LIDAT", 36.95, -117.45);
        put(points, "OAL", 38.00, -117.77);
        put(points, "ECA", 38.00, -120.80);
        put(points, "SUPER", 38.10, -123.00);
        put(points, "BEBOP", 38.20, -124.50);
        put(points, "BAART", 38.50, -128.00);
        put(points, "BARAZ", 39.00, -135.00);
        put(points, "BILLO", 39.50, -145.00);
        put(points, "BEKME", 40.00, -155.00);
        put(points, "BOARD", 38.00, -158.00);
        put(points, "BITTA", 32.00, -157.00);
        put(points, "HNL", 21.31, -157.92);
        put(points, "FYTTR", 36.20, -114.80);
        put(points, "FMG", 39.53, -119.66);
        put(points, "BAARB", 40.00, -120.20);
        put(points, "LKV", 42.16, -120.40);
        put(points, "UBG", 45.35, -122.98);
        put(points, "ARRIE", 47.00, -124.00);
        put(points, "TOU", 48.30, -124.63);
        put(points, "YZT", 50.68, -127.37);
        put(points, "DUGGS", 53.00, -133.00);
        put(points, "HANRY", 55.00, -136.00);
        put(points, "ANN", 55.04, -131.57);
        put(points, "LVD", 56.47, -133.10);
        put(points, "SSR", 58.17, -135.25);
        put(points, "YAK", 59.50, -139.66);
        put(points, "JOH", 60.48, -146.60);
        put(points, "ANC", 61.17, -150.00);
        put(points, "EDF", 61.25, -149.80);
        return points;
    }

    private static void put(Map<String, GeoCoordinate> points, String name, double latitude, double longitude) {
        points.put(name, GeoCoordinate.builder().latitude(latitude).longitude(longitude).altitude(0).build());
    }
}
