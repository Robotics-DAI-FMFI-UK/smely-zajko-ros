#include "LocalizationAndPlanning.h"

LocalizationAndPlanning::LocalizationAndPlanning(int guiWidth, int guiHeight) {
    this->guiDebugHeight = 50; // debug border height 0to turn off
    // this->guiDebugHeight = 0;
    this->guiMapWidth = guiWidth;
    this->guiMapHeight = guiHeight - this->guiDebugHeight;
    this->guiDebugWidth = guiWidth;
    lastPosition.latitude = 0;
    lastPosition.longitude = 0;
    curPoint.latitude = 0;
    curPoint.longitude = 0;
    ell_a = 0;
    ell_b = 0;
    EarthRadius = 6376.5;
    heading_search_radius = 0.006;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.3, 0.3);
    cvInitFont(&fontBig, CV_FONT_HERSHEY_SIMPLEX, 0.6, 0.6);
}

LocalizationAndPlanning::~LocalizationAndPlanning() {}

geometry_msgs::Point LocalizationAndPlanning::convert(sensor_msgs::NavSatFix point) {

    double width = bounds.maxlon - bounds.minlon;
    double height = bounds.maxlat - bounds.minlat;

    geometry_msgs::Point result;

    result.x = ((point.longitude - bounds.minlon) / width) * guiMapWidth;
    result.y = ((1 - (point.latitude - bounds.minlat) / height)) * guiMapHeight;

    return result;
}

sensor_msgs::NavSatFix LocalizationAndPlanning::reverse(geometry_msgs::Point location) {

    sensor_msgs::NavSatFix result;
    result.longitude = location.x / (double) guiMapWidth *
                       (bounds.maxlon - bounds.minlon) +
                       bounds.minlon;
    result.latitude = location.y / (double) guiMapHeight *
                      (bounds.maxlat - bounds.minlat) +
                      bounds.minlat;
    return result;
}

void LocalizationAndPlanning::readMap(char *filename) {

    ifstream is;
    is.open(filename);

    char *str;

    if (!is.good()) {
        ROS_ERROR("cannot read map file \n");

        return;
    }

    // get length of file:
    is.seekg(0, ios::end);
    long length = is.tellg();
    is.seekg(0, ios::beg);

    str = new char[length + 1];
    for (int i = 0; i < length + 1; i++) {
        str[i] = 0;
    }

    is.read(str, length);

    is.close();

    // printf("%s", str);

    xml_document<> doc; // character type defaults to char
    doc.parse<0>(str);  // 0 means default parse flags

    xml_node<> *base_node = doc.first_node("osm");

    // bounds
    xml_node<> *bounds_node = base_node->first_node("bounds");

    istringstream isstr1(bounds_node->first_attribute("minlat")->value());
    isstr1 >> bounds.minlat;

    istringstream isstr2(bounds_node->first_attribute("maxlat")->value());
    isstr2 >> bounds.maxlat;

    istringstream isstr3(bounds_node->first_attribute("minlon")->value());
    isstr3 >> bounds.minlon;

    istringstream isstr4(bounds_node->first_attribute("maxlon")->value());
    isstr4 >> bounds.maxlon;

    // find footways
    for (xml_node<> *way_node = base_node->first_node("way"); way_node;
         way_node = way_node->next_sibling("way")) {
        int candidate = 0;
        for (xml_node<> *tag_node = way_node->first_node("tag"); tag_node;
             tag_node = tag_node->next_sibling("tag")) {

            xml_attribute<> *attr_v = tag_node->first_attribute("v");

            if (attr_v == 0) {
                continue;
            }

            if ((strcmp(attr_v->value(), "footway") == 0) ||
                (strcmp(attr_v->value(), "residential") == 0) ||
                (strcmp(attr_v->value(), "path") == 0) ||
                (strcmp(attr_v->value(), "unclassified") == 0) ||
                (strcmp(attr_v->value(), "pedestrian") == 0) ||
                (strcmp(attr_v->value(), "service") == 0)) {
                candidate++;
            }

            if (strcmp(attr_v->value(), "sand") == 0)
                candidate--;
            if (strcmp(attr_v->value(), "grass") == 0)
                candidate--;
        }
        if (candidate > 0) {
            Path path;

            // read all nodes
            for (xml_node<> *nd_node = way_node->first_node("nd"); nd_node;
                 nd_node = nd_node->next_sibling("nd")) {
                xml_attribute<> *attr_ref = nd_node->first_attribute("ref");

                if (attr_ref == 0) {
                    continue;
                }

                // search in 'osm' node for 'node' elements
                for (xml_node<> *node = base_node->first_node("node"); node;
                     node = node->next_sibling("node")) {

                    xml_attribute<> *attr_id = node->first_attribute("id");
                    if (attr_id == 0) {
                        continue;
                    }

                    if (strcmp(attr_id->value(), attr_ref->value()) == 0) {

                        WayPoint point;

                        istringstream istr1(
                                node->first_attribute("lat")->value());
                        istr1 >> point.latitude;

                        istringstream istr2(
                                node->first_attribute("lon")->value());
                        istr2 >> point.longitude;

                        double id = atof(node->first_attribute("id")->value());

                        points[id] = point;

                        path.points.push_back(id);
                    }
                }
            }

            paths.push_back(path);
        }
    }

    doc.clear();

    delete[] str;

    for (int i = 0; i < paths.size(); i++) {
        unsigned long points_size = paths[i].points.size();

        // if the path does not have points on the map
        // we do not consider it to be valid
        if (points_size == 0) {
            continue;
        }

        for (int j = 1; j < points_size - 1; j++) {
            double id1 = paths[i].points[j];
            double id2 = paths[i].points[j - 1];
            double id3 = paths[i].points[j + 1];

            points[id1].nextPoints.push_back(id2);
            points[id1].nextPoints.push_back(id3);
        }
        // first
        double id0 = paths[i].points[0];
        double id1 = paths[i].points[1];

        points[id0].nextPoints.push_back(id1);

        if (paths[i].points.size() >= 2) {
            // last
            id0 = paths[i].points[paths[i].points.size() - 1];
            id1 = paths[i].points[paths[i].points.size() - 2];

            points[id0].nextPoints.push_back(id1);
        }
    }
}

void LocalizationAndPlanning::readDestination(char *filename) {

    sensor_msgs::NavSatFix point;

    ifstream is;
    is.open(filename);

    char buff[128];

    while (is.good()) {
        is.getline(buff, 128);

        string s(buff);
        string s2;

        istringstream iss(s);

        iss >> s2;

        if (s2 == "latitude") {
            iss >> point.latitude;
        } else if (s2 == "longitude") {
            iss >> point.longitude;
        }
    }

    setDestination(point);
}

void LocalizationAndPlanning::setDestination(sensor_msgs::NavSatFix point) {

    FindOnWay fw;
    fw = find_on_way(point);
    destinationPoint = fw.pointFound;

    // vypocitaj elipsu okolia pre danu geopoziciu
    calcEllipse(point, heading_search_radius);
}

// vypocita ell_a ell_b parametre pre elipsu v bode
// point s radiusom km kilometrov
void LocalizationAndPlanning::calcEllipse(sensor_msgs::NavSatFix point, double km) {
    double preccons = 0.0001;
    sensor_msgs::NavSatFix dummy;

    // zisti deltu pre lat
    dummy.latitude = point.latitude + preccons;
    dummy.longitude = point.longitude;
    double delta = distance(point, dummy);

    ell_a = (km / delta) * preccons;

    // zisti deltu pre lon
    dummy.latitude = point.latitude;
    dummy.longitude = point.longitude + preccons;
    delta = distance(point, dummy);

    ell_b = (km / delta) * preccons;
}

// prienik usecky a elipsy ak nema prienik tj aj ked je to
// dotyk vrati lon a lat DBL_MAX
// x = lat y = lon
pair<sensor_msgs::NavSatFix, sensor_msgs::NavSatFix>
LocalizationAndPlanning::ellipseLineIntersection(sensor_msgs::NavSatFix p0, sensor_msgs::NavSatFix p1) {
    sensor_msgs::NavSatFix re1;
    sensor_msgs::NavSatFix re2;
    // move so ellipse center is in origin
    p0.latitude -= curPoint.latitude;
    p0.longitude -= curPoint.longitude;
    p1.latitude -= curPoint.latitude;
    p1.longitude -= curPoint.longitude;

    // calc line slope
    double latdif = p1.latitude - p0.latitude;
    if (latdif == 0)
        latdif = DBL_MIN;
    double m = (p1.longitude - p0.longitude) / latdif;

    double c = p1.longitude - m * p1.latitude;
    // pomocne mocniny
    double a_2 = ell_a * ell_a;
    double b_2 = ell_b * ell_b;
    double c_2 = c * c;
    double m_2 = m * m;
    // prienik elipsy a priamky
    double hlp1 = (2 * a_2 * c * m) / (2 * (b_2 + a_2 * m_2));
    double hlp3 = ((b_2 - c_2) / (b_2 + a_2 * m_2)) +
                  ((a_2 * c_2 * m_2) / (pow((b_2 + a_2 * m_2), 2)));
    double hlp2 = 0;
    if (hlp3 > 0) { // realna cast odmocniny zaporneho cisla je nenulova ak je
        // cislo kladne
        hlp2 = ell_a * sqrt(hlp3);
    }
    re1.latitude = -hlp1 + hlp2;
    re1.longitude = m * (re1.latitude) + c;
    re2.latitude = -hlp1 - hlp2;
    re2.longitude = m * (re2.latitude) + c;

    // posun body spat
    re1.latitude += curPoint.latitude;
    re1.longitude += curPoint.longitude;
    re2.latitude += curPoint.latitude;
    re2.longitude += curPoint.longitude;
    p0.latitude += curPoint.latitude;
    p0.longitude += curPoint.longitude;
    p1.latitude += curPoint.latitude;
    p1.longitude += curPoint.longitude;

    // nieje prienik
    if (re1.latitude == re2.latitude && re1.longitude == re2.longitude) {
        re1.latitude = DBL_MAX;
        re1.longitude = DBL_MAX;
        re2.latitude = DBL_MAX;
        re2.longitude = DBL_MAX;
        return pair<sensor_msgs::NavSatFix, sensor_msgs::NavSatFix>(re1, re2);
    }

    // check ci je na usecke a nie mimo
    double seglen = distance(p0, p1);
    if (distance(re1, p0) > seglen || distance(re1, p1) > seglen) {
        re1.latitude = DBL_MAX;
        re1.longitude = DBL_MAX;
    }
    if (distance(re2, p0) > seglen || distance(re2, p1) > seglen) {
        re2.latitude = DBL_MAX;
        re2.longitude = DBL_MAX;
    }

    return pair<sensor_msgs::NavSatFix, sensor_msgs::NavSatFix>(re1, re2);
}

sensor_msgs::NavSatFix LocalizationAndPlanning::calcHeadingPoint() {

    if (bestWay.size() < 3) {
        // if only one segment remains and destination in range mark it as
        // heading point
        if (distance(destinationPoint, curPoint) < heading_search_radius)
            return destinationPoint;
        // bestway is oriented so if we went behind destination swap bestway
        // orientation
        if (bestWay.size() == 2 &&
            distance(points[bestWay[0]], curPoint) <
            distance(destinationPoint, points[bestWay[0]])) {
            double swp = bestWay[1];
            bestWay[1] = bestWay[0];
            bestWay[0] = swp;
        }
    }

    // init
    sensor_msgs::NavSatFix result;
    result.latitude = DBL_MAX;
    result.longitude = DBL_MAX;

    bool out = false; // vychadza z radiusu
    // get heading point as intersection of bestway and elliptical radius
    // best way
    if (bestWay.size() > 0) {
        // vyber bod na prieniku elipsy a najskorsieho segmentu  vychadzajuceho
        // z ellipsy
        sensor_msgs::NavSatFix p1 = points[bestWay[bestWay.size() - 1]];
        for (unsigned long i = bestWay.size() - 2; i >= 0; i--) {
            sensor_msgs::NavSatFix p2 = points[bestWay[i]];
            // vzdialenost polohy od konca segmentu
            double tmpdst = distance(curPoint, p2);

            pair<sensor_msgs::NavSatFix, sensor_msgs::NavSatFix> par = ellipseLineIntersection(p1, p2);
            // ak su dva prieniky alebo zacinajuci bod lezi dnu a existuje
            // prienik musi byt vychadzajuci prienik
            if ((par.first.longitude != DBL_MAX &&
                 par.second.longitude != DBL_MAX) ||
                ((distance(p1, curPoint) < heading_search_radius) &&
                 (par.first.longitude != DBL_MAX ||
                  par.second.longitude != DBL_MAX))) {
                out = true;
            }
            if (out) { // dalsi segment vychadza
                // zober najlepsi validny na tomto segmente
                if (par.first.longitude != DBL_MAX &&
                    distance(par.first, p2) < tmpdst) {
                    result = par.first;
                    break;
                }

                if (par.second.longitude != DBL_MAX &&
                    distance(par.second, p2) < tmpdst) {
                    result = par.second;
                    break;
                }
            }
            p1 = p2;
        }
    }

    // if no intersection is found it means all segments are in radius
    if (result.latitude == DBL_MAX) {

        if (bestWay.size() > 2) { // set the firsts segments second point(the
            // one shared with second segment) a he gets
            // close to that intersection he will most
            // likely catch on
            result = points[bestWay[bestWay.size() - 2]];
        } else { // if only one segment remains set destination as heading point
            result = destinationPoint;
        }
    }

    return result;
}

IplImage *LocalizationAndPlanning::getGui() {
    IplImage *result = cvCreateImage(
            cvSize(guiMapWidth, guiMapHeight + guiDebugHeight), 32, 3);
    // biele pozadie
    cvSet(result, CV_RGB(255, 255, 255));
    // cesty
    for (unsigned long i = 0; i < paths.size(); i++) {
        if (paths.at(i).points.size() > 1) {
            geometry_msgs::Point p1 = convert(points.at(paths.at(i).points[0]));

            for (int j = 1; j < paths.at(i).points.size(); j++) {
                geometry_msgs::Point p2 = convert(points.at(paths.at(i).points[j]));

                cvLine(result, cvPoint(p1.x, p1.y), cvPoint(p2.x, p2.y),
                       cvScalar(0, 0, 0));
                p1 = p2;
            }
        }
    }

    // best way
    if (bestWay.size() > 0) {
        double bluuu = (double) 1 / bestWay.size(); // gradient
        geometry_msgs::Point p1 = convert(points[bestWay[0]]);
        for (int i = 1; i < bestWay.size(); i++) {
            geometry_msgs::Point p2 = convert(points[bestWay[i]]);
            cvLine(result, cvPoint(p1.x, p1.y), cvPoint(p2.x, p2.y),
                   cvScalar(0.5 + 0.5 * (i * (bluuu)), 0, 0), 2);
            p1 = p2;
        }
    }

    // raw gps - kruh
    geometry_msgs::Point o2 = convert(lastPosition);
    cvCircle(result, cvPoint(o2.x, o2.y), 3, cvScalar(0, 1, 0), -1);

    // ciel - stvorcek
    geometry_msgs::Point o3 = convert(destinationPoint);
    cvRectangle(result, cvPoint(o3.x - 5, o3.y - 5),
                cvPoint(o3.x + 5, o3.y + 5), cvScalar(0.8, 0.8, 0.8), -1);

    // poloha - krizik
    geometry_msgs::Point o4 = convert(curPoint);
    cvLine(result, cvPoint(o4.x - 4, o4.y - 4), cvPoint(o4.x + 4, o4.y + 4),
           cvScalar(0, 0, 1), 2);
    cvLine(result, cvPoint(o4.x - 4, o4.y + 4), cvPoint(o4.x + 4, o4.y - 4),
           cvScalar(0, 0, 1), 2);

    // medziciel - plny kruh
    geometry_msgs::Point o1 = convert(headingPoint);
    cvCircle(result, cvPoint(o1.x, o1.y), 3, cvScalar(1, 0, 1), -1);

    // elipsa okolia
    sensor_msgs::NavSatFix dummy;
    dummy.longitude = curPoint.longitude;
    dummy.latitude = curPoint.latitude + ell_a;
    geometry_msgs::Point o8 = convert(dummy);
    dummy.longitude = curPoint.longitude + ell_b;
    dummy.latitude = curPoint.latitude;
    geometry_msgs::Point o9 = convert(dummy);
    cvEllipse(result, cvPoint(o4.x, o4.y),
              cvSize(abs(o4.x - o9.x), abs(o4.y - o8.y)), 0, 0, 360,
              cvScalar(0.5, 0.5, 0.5));

    // draw debug
    if (guiDebugHeight > 0) {
        // base
        cvRectangle(result, cvPoint(0, guiMapHeight),
                    cvPoint(guiDebugWidth, guiMapHeight + guiDebugHeight),
                    cvScalar(1, 1, 1), -1);
        cvLine(result, cvPoint(0, guiMapHeight),
               cvPoint(guiDebugWidth, guiMapHeight), cvScalar(0, 0, 0), 1);

        // distances
        cvPutText(result, "gps", cvPoint(5, guiMapHeight + 15), &font,
                  cvScalar(0, 0, 0));
        cvCircle(result, cvPoint(30, guiMapHeight + 15), 3, cvScalar(0, 1, 0),
                 -1);
        cvPutText(result, "->curPos", cvPoint(35, guiMapHeight + 15), &font,
                  cvScalar(0, 0, 0));
        cvLine(result, cvPoint(90 - 4, guiMapHeight + 15 - 4),
               cvPoint(90 + 4, guiMapHeight + 15 + 4), cvScalar(0, 0, 1), 2);
        cvLine(result, cvPoint(90 - 4, guiMapHeight + 15 + 4),
               cvPoint(90 + 4, guiMapHeight + 15 - 4), cvScalar(0, 0, 1), 2);

        stringstream diststr;
        diststr << distance(lastPosition, curPoint) * 1000 << "m";
        cvPutText(result, diststr.str().c_str(), cvPoint(5, guiMapHeight + 40),
                  &fontBig, cvScalar(0, 0, 0));

        cvPutText(result, "curPos", cvPoint(120, guiMapHeight + 15), &font,
                  cvScalar(0, 0, 0));
        cvLine(result, cvPoint(160 - 4, guiMapHeight + 15 - 4),
               cvPoint(160 + 4, guiMapHeight + 15 + 4), cvScalar(0, 0, 1), 2);
        cvLine(result, cvPoint(160 - 4, guiMapHeight + 15 + 4),
               cvPoint(160 + 4, guiMapHeight + 15 - 4), cvScalar(0, 0, 1), 2);
        cvPutText(result, "->dest", cvPoint(165, guiMapHeight + 15), &font,
                  cvScalar(0, 0, 0));
        cvRectangle(result, cvPoint(207 - 5, guiMapHeight + 15 - 5),
                    cvPoint(207 + 5, guiMapHeight + 15 + 5),
                    cvScalar(0.8, 0.8, 0.8), -1);

        diststr.str("");
        diststr << distance(destinationPoint, curPoint) * 1000 << "m";
        cvPutText(result, diststr.str().c_str(),
                  cvPoint(120, guiMapHeight + 40), &fontBig, cvScalar(0, 0, 0));
        // bestway info
        diststr.str("seg");
        cvPutText(result, diststr.str().c_str(),
                  cvPoint(340, guiMapHeight + 20), &font, cvScalar(0, 0, 0));
        diststr.str("");
        diststr << bestWay.size() - 1;
        cvPutText(result, diststr.str().c_str(),
                  cvPoint(335, guiMapHeight + 40), &fontBig, cvScalar(0, 0, 0));
    }

    return result;
}

double LocalizationAndPlanning::distance(sensor_msgs::NavSatFix p1, sensor_msgs::NavSatFix p2) {
    // printf("p1=%lf, %lf   p2=%lf, %lf\n", p1.latitude, p1.longitude,
    // p2.latitude, p2.longitude);

    // degrees to radians
    p1.latitude *= (M_PI / 180);
    p1.longitude *= (M_PI / 180);
    p2.latitude *= (M_PI / 180);
    p2.longitude *= (M_PI / 180);

    double dlon = p2.longitude - p1.longitude;
    double dlat = p2.latitude - p1.latitude;
    // earth radius * (radial distance)
    return EarthRadius * 2 * asin(sqrt(pow(sin(dlat / 2), 2) +
                                       cos(p1.latitude) * cos(p2.latitude) *
                                       pow(sin(dlon / 2), 2)));
}

// vypocita initial bearing z bodu a do bodu b
double LocalizationAndPlanning::calc_bearing(sensor_msgs::NavSatFix a, sensor_msgs::NavSatFix b) {
    // deg to rad

    a.longitude *= (M_PI / 180);
    a.latitude *= (M_PI / 180);
    b.longitude *= (M_PI / 180);
    b.latitude *= (M_PI / 180);
    double res = atan2(sin(b.longitude - a.longitude) * cos(b.latitude),
                       cos(a.latitude) * sin(b.latitude) -
                       sin(a.latitude) * cos(b.latitude) *
                       cos(b.longitude - a.longitude));
    // we want positive degrees
    res /= (M_PI / 180);
    res += 360;
    res = fmod(res, 360);
    return res;
}

// ak su rovnake vrati bod p1 ak je nejasne vrati p2
sensor_msgs::NavSatFix
LocalizationAndPlanning::intersection_of_bearings(sensor_msgs::NavSatFix p1, double b1, sensor_msgs::NavSatFix p2,
                                                  double b2) {

    double lat1 = p1.latitude * (M_PI / 180);
    double lon1 = p1.longitude * (M_PI / 180);
    double lat2 = p2.latitude * (M_PI / 180);
    double lon2 = p2.longitude * (M_PI / 180);

    double t13 = b1 * (M_PI / 180);
    double t23 = b2 * (M_PI / 180);
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    // radial distance of p1 - p2
    double D12 = 2 * asin(sqrt(pow(sin(dlat / 2), 2) +
                               cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2)));

    if (D12 == 0)
        return p1; // p1 and p2 are the same point

    // initial/final bearings between points
    double t1 = 0;
    double pom = sin(D12) * cos(lat1);
    double pom2 = 666; // anything>1 for acos check
    if (pom != 0) {
        pom2 = (sin(lat2) - sin(lat1) * cos(D12)) / pom;
    }
    if (abs(pom2) <= 1) {
        t1 = acos(pom2);
    }

    double t2 = 0;
    pom = sin(D12) * cos(lat2);
    pom2 = 666; // anything>1 for acos check
    if (pom != 0) {
        pom2 = (sin(lat1) - sin(lat2) * cos(D12)) / pom;
    }
    if (abs(pom2) <= 1) {
        t2 = acos(pom2);
    }

    double t12 = 0;
    double t21 = 0;
    if (sin(lon2 - lon1) > 0) {
        t12 = t1;
        t21 = 2 * M_PI - t2;
    } else {
        t12 = 2 * M_PI - t1;
        t21 = t2;
    }

    double a1 = fmod((t13 - t12 + M_PI), (2 * M_PI)) - M_PI; // angle 2-1-3
    double a2 = fmod((t21 - t23 + M_PI), (2 * M_PI)) - M_PI; // angle 1-2-3

    if (sin(a1) == 0 && sin(a2) == 0)
        return p1; // infinite intersections
    if (sin(a1) * sin(a2) < 0)
        return p2; // ambiguous intersection

    double a3 = acos(-cos(a1) * cos(a2) + sin(a1) * sin(a2) * cos(D12));
    double D13 =
            atan2(sin(D12) * sin(a1) * sin(a2), cos(a2) + cos(a1) * cos(a3));
    double lat3 = asin(sin(lat1) * cos(D13) + cos(lat1) * sin(D13) * cos(t13));
    double dlon13 = atan2(sin(t13) * sin(D13) * cos(lat1),
                          cos(D13) - sin(lat1) * sin(lat3));
    double lon3 = lon1 + dlon13;
    lon3 = fmod((lon3 + 3 * M_PI), (2 * M_PI)) - M_PI;

    sensor_msgs::NavSatFix intersec;
    intersec.latitude = lat3 * (180 / M_PI);
    intersec.longitude = lon3 * (180 / M_PI);

    return intersec;
}

// vypocita vzdialenost usecky start-end a bodu "point" a najblizsi bod usecky k
// "point"
pair<double, sensor_msgs::NavSatFix>
LocalizationAndPlanning::dist_point_linesegment(sensor_msgs::NavSatFix point, sensor_msgs::NavSatFix start,
                                                sensor_msgs::NavSatFix end) {

    sensor_msgs::NavSatFix closest;
    double b_se = calc_bearing(start, end);
    double b_sp = calc_bearing(start, point);

    // assume we are right of line
    double pravyuhol = -90;
    // check if we are left of line
    if (fmod(360 + b_sp - b_se, 360) > 180)
        pravyuhol = 90;

    double b_perp = b_se + pravyuhol;

    // calc intersection of trajectories
    closest = intersection_of_bearings(point, b_perp, start, b_se);

    // closest is the closest point on great circle - trim it to segment
    double cls = distance(closest, start);
    double cle = distance(closest, end);
    double seglen = distance(start, end);

    if (cls > seglen || cle > seglen) {
        if (distance(start, point) > distance(end, point))
            closest = end;
        else
            closest = start;
    }

    return pair<double, sensor_msgs::NavSatFix>(distance(closest, point), closest);
}

FindOnWay LocalizationAndPlanning::find_on_way(sensor_msgs::NavSatFix point) {
    FindOnWay result = FindOnWay();
    long double dist = DBL_MAX;
    pair<double, sensor_msgs::NavSatFix> p;
    for (unsigned long i = 0; i < paths.size(); i++) {
        for (int j = 1; j < paths.at(i).points.size(); j++) {
            double id1 = paths.at(i).points[j];
            double id2 = paths.at(i).points[j - 1];
            p = dist_point_linesegment(point, points.at(id1), points.at(id2));
            if (p.first < dist) {
                dist = p.first;
                result.pointFound = p.second;
                result.pathId = i;
                result.pointId1 = id1;
                result.pointId2 = id2;
                result.pathPosition = j;
            }
        }
    }

    return result;
}

bool distCompare(IdDist i, IdDist j) { return (i.dist < j.dist); }

void LocalizationAndPlanning::calcPath(
        double strtPoint, double strtPointB, double destPoint,
        double destPointB) { // najde najkratsiu cestu z cesty strt na cestu dest

    set<double> usedPoints;
    list<double> toProcess;

    map<double, WayPoint>::iterator it;
    for (it = points.begin(); it != points.end(); it++) {
        (*it).second.dist = DBL_MAX;
    }

    toProcess.push_back(strtPoint);

    points[strtPoint].dist = 0;

    // process points
    while (toProcess.size() > 0) {
        double id = toProcess.front();
        toProcess.pop_front();

        WayPoint w = points[id];

        vector<IdDist> toProcess2;

        for (int i = 0; i < w.nextPoints.size(); i++) {

            double idn = w.nextPoints[i];
            double dist = w.dist + distance(w, points[idn]);

            if (dist < points[idn].dist) {
                IdDist tmp;
                tmp.dist = dist;
                tmp.id = idn;
                toProcess2.push_back(tmp);
            }
        }

        sort(toProcess2.begin(), toProcess2.end(), distCompare);

        vector<IdDist>::iterator it2;
        for (it2 = toProcess2.begin(); it2 != toProcess2.end(); it2++) {
            toProcess.push_back((*it2).id);
            points[(*it2).id].dist = (*it2).dist;
            points[(*it2).id].previous = id;
        }
    }

    // read path
    usedPoints.clear();
    toProcess.clear();

    toProcess.push_back(destPoint);
    vector<double> path;
    path.push_back(destPoint);

    while (toProcess.size() > 0) {
        double id = toProcess.front();
        toProcess.pop_front();

        if (id == strtPoint) {
            path.push_back(strtPoint);
            break;
        }

        WayPoint w = points[id];
        if (w.previous != -1 && usedPoints.count(w.previous) == 0) {
            path.push_back(w.previous);
            toProcess.push_back(w.previous);
        }
        usedPoints.insert(id);
    }
    bestWay.clear();
    // ak nieje finalny bod cesty na kt lezi ciel vo vypocitanej ceste pridame
    // ho
    if (path.size() < 2 || path[1] != destPointB) {
        bestWay.push_back(destPointB);
    }
    for (int i = 0; i < path.size(); i++) {
        // dont allow 0distance steps
        if (bestWay.size() == 0 || path[i] != bestWay[bestWay.size() - 1])
            bestWay.push_back(path[i]);
    }
    // ak nieje zaciatok cesty kde sa nachadzame v bestway pridaj
    if (bestWay[bestWay.size() - 2] != strtPointB &&
        bestWay[bestWay.size() - 1] != strtPointB) {
        bestWay.push_back(strtPointB);
    }
}

message_types::GpsAngles LocalizationAndPlanning::update(sensor_msgs::NavSatFix gps) {

    message_types::GpsAngles result;

    lastPosition = gps;

    FindOnWay fw;
    // najdime sa na nejakej ceste
    fw = find_on_way(gps);
    curPoint = fw.pointFound;

    // ak sme uz na druhom segmente odstranime ten prvy

    if (bestWay.size() > 2 && ((fw.pointId1 == bestWay[bestWay.size() - 2] &&
                                fw.pointId2 == bestWay[bestWay.size() - 3]) ||
                               (fw.pointId1 == bestWay[bestWay.size() - 3] &&
                                fw.pointId2 == bestWay[bestWay.size() - 2]))) {
        // remove first segment
        bestWay.erase(bestWay.end() - 1);
    }
    // TODO presne kedy recalc doriesit  teraz solidne ale ak sa vyberie jednym
    // smerom dlho tak sa da pokazit- mozno recalc ak nemame heading point ako
    // priesecnik alebo destpoint tj ten druhy bod  prveho segmentu
    // ak nemame trasu alebo sme na ceste ktora sa nespaja s bestway na jej
    // zaciatku -> vypocitame trasu
    if (bestWay.size() == 0 ||
        (bestWay.size() == 1 && fw.pointId1 != bestWay[bestWay.size() - 1] &&
         fw.pointId2 != bestWay[bestWay.size() - 1]) ||
        (bestWay.size() > 1 && fw.pointId1 != bestWay[bestWay.size() - 1] &&
         fw.pointId2 != bestWay[bestWay.size() - 1] &&
         fw.pointId1 != bestWay[bestWay.size() - 2] &&
         fw.pointId2 != bestWay[bestWay.size() - 2])) {
        FindOnWay fwDest = find_on_way(destinationPoint);
        calcPath(fw.pointId1, fw.pointId2, fwDest.pointId1, fwDest.pointId2);
        // printf("new bestWay calculated\n");
    }

    // najdeme headingpoint
    headingPoint = calcHeadingPoint();

    // checkneme ciel a vratime data
    result.dstToFin = distance(destinationPoint, curPoint);
    result.dstToHeadingPoint = distance(lastPosition, curPoint);

    // bearing in degrees to heading point
    result.map = calc_bearing(curPoint, headingPoint);

    if (result.dstToFin < 0.002) { // dst v km
        printf("SME V CIELI ( %f m ) \n", result.dstToFin * 1000);
        result.map = DBL_MAX;
    }
    return result;
};