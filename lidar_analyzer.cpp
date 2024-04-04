#include "lidar_analyzer.h"
#include "lidar.h"

// const int LONGUEUR_TERRAIN = 2430;
const int LARGEUR_TERRAIN = 1820;
const double HoughTransformMemorySize = 16000.0;
const double rhoTolerance = 500.0; 
const double thetaMargin = 0.5;
const double thetaToleranceParallel = 0.2;
const double thetaTolerancePerpendiculaire = 0.2;

Vector3 rotateVector(Vector3 vec, double rad) {
    //double rad = theta * M_PI / 180.0; // Convertir en radians
    return {
        vec.x * cos(rad) - vec.y * sin(rad),
        vec.x * sin(rad) + vec.y * cos(rad)
    };
}

// Transformation d'un point du repère (x, y) au repère (u, v)
Vector3 transformToUV(Vector3 M, Vector3 O, double theta) {
    // Calculer les vecteurs de direction de u et v
    Vector3 u_dir = rotateVector({1, 0}, -theta); // Rotation de -theta
    Vector3 v_dir = rotateVector({0, 1}, -theta); // Rotation de -theta

    // Obtenir les coordonnées de O par rapport à M (le centre du rectangle)
    Vector3 OM = {O.x - M.x, O.y - M.y};

    // Projeter OM sur u et v pour obtenir les coordonnées dans le repère (u, v)
    Vector3 O_uv = {
        OM.x * u_dir.x + OM.y * u_dir.y, // Produit scalaire pour u
        OM.x * v_dir.x + OM.y * v_dir.y  // Produit scalaire pour v
    };

    return O_uv;
}

Vector3 computeCentroid(const std::vector<Vector3>& corners) {
    Vector3 centroid = {0, 0};
    for (const auto& corner : corners) {
        centroid.x += corner.x;
        centroid.y += corner.y;
    }
    centroid.x /= corners.size();
    centroid.y /= corners.size();
    return centroid;
}

// Fonction pour ordonner les coins dans le sens des aiguilles d'une montre
void sortPointsClockwise(std::vector<Vector3>& points, Vector3 center) {
    std::sort(points.begin(), points.end(), [center](const Vector3& a, const Vector3& b) {
        double angleA = std::atan2(a.y - center.y, a.x - center.x);
        double angleB = std::atan2(b.y - center.y, b.x - center.x);

        // Normalisation des angles pour s'assurer qu'ils sont dans le même intervalle
        angleA = std::fmod(angleA, 2 * M_PI);
        angleB = std::fmod(angleB, 2 * M_PI);

        // Comparaison des angles normalisés
        return angleA > angleB;
    });
}

double calculateAngle(Vector3 center, Vector3 robot) {
    double deltaX = robot.x - center.x;
    double deltaY = robot.y - center.y;
    return std::atan2(deltaY, deltaX);
}

std::vector<Line> houghTransform(const std::vector<Vector2>& points, int numRho, int numTheta, double thetaStep, int threshold) {
    std::vector<Line> lines;
    double rhoStep = numRho * 2 * numTheta / HoughTransformMemorySize;
    std::vector<int> accumulator(HoughTransformMemorySize, 0);
    // SerialDebug.println("accumulator: " + String(accumulator.size()));

    for (const auto& point : points) {
      // SerialDebug.println("point : x=" + String(point.x()) + ", y=" + String(point.y()));
        for (int thetaIndex = 0; thetaIndex < numTheta; thetaIndex++) {
            double theta = thetaIndex * thetaStep;
            int rhoIndex = round((numRho + (point.x() * cos(theta) + point.y() * sin(theta))) / rhoStep);
            int accuIndex = thetaIndex * numRho * 2 / rhoStep + rhoIndex;
            // SerialDebug.println("theta=" + String(theta) + ", rhoIndex=" + String(rhoIndex) 
                // + ", accu index=" + String(accuIndex));
            if (accuIndex >= 0 && accuIndex < accumulator.size()) {
                accumulator[accuIndex]++;
                if(accuIndex == 428) {
                  // SerialDebug.println("rhoIndex=" + String(rhoIndex) + ", theta=" + String(theta) + ", rho=" +
                  //     String((point.x() * cos(theta) + point.y() * sin(theta))) + ", temp1=" +
                  //     String((point.x() * cos(theta) + point.y() * sin(theta)) / rhoStep) + ", temp2=" +
                  //     String((numRho / rhoStep) + ((point.x() * cos(theta) + point.y() * sin(theta)) / rhoStep)) + ", temp3=" +
                  //     String((numRho + (point.x() * cos(theta) + point.y() * sin(theta))) / rhoStep) +
                  //     ", thetaIndex=" + String(thetaIndex));
                }
            }
        }
    }

    int max_threshold = 0;
    for (int thetaIndex = 0; thetaIndex < numTheta; thetaIndex++) {
      for (int rhoIndex = 0; rhoIndex < numRho * 2 / rhoStep; rhoIndex++) {
         int indice = thetaIndex * numRho * 2 / rhoStep + rhoIndex;
          
          if(accumulator[indice] > max_threshold) {
            max_threshold = accumulator[indice];
          }

          if (accumulator[indice] >= threshold) {
              Line line;
              // SerialDebug.println("rhoIndex=" + String(rhoIndex) + ", indice=" + String(indice) + ", thetaIndex="
              //     + String(thetaIndex) + ", rho=" + String(rhoIndex * rhoStep - numRho));
              line.rho = rhoIndex * rhoStep - numRho;
              line.theta = thetaIndex * thetaStep;
              line.nb_accumulators = accumulator[indice];
              lines.push_back(line);
          }
        }
    }
    // SerialDebug.println("max threshold=" + String(max_threshold));

    return lines;
}

float distance(const Vector2& p1, const Vector2& p2) {
    return std::sqrt(std::pow(p2.x() - p1.x(), 2) + std::pow(p2.y() - p1.y(), 2));
}

float distance3(const Vector3& p1, const Vector3& p2) {
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

double calculateX(const Line& wall) {
    return wall.rho / cos(wall.theta);
}

RobotInfos getFieldInfos(bool readFromLidar = true, bool show_log = false) {  
  RobotInfos infos;
  std::vector<LidarPoint> points;
  double distance_max = 0;
  String full_log = "****************\r\n";
  std::vector<Vector2> points_cart;
  unsigned long start_millis = millis();

  if(readFromLidar) {
    
    CircularLidarPointsBuffer lidarPointsBuffer = CircularLidarPointsBuffer(456);
      
    const int nb_tours_lidar = 55;

    for (int i = 0; i < nb_tours_lidar; i++) {
        std::vector<LidarPoint> lidarPoints = lidarPointsBuffer.getPoints();
        for (size_t j = 0; j < lidarPoints.size(); j++) {
          LidarPoint lidarPoint = lidarPoints[j];
          if(lidarPoint.distance() > 130) { // on ne prend pas les points < 13cm
            points.push_back(lidarPoint); 
            if(lidarPoint.distance() > distance_max) {
              distance_max = lidarPoint.distance();
            }        
          }
        }
    }

    unsigned long elapsed = millis() - start_millis;
    // SerialDebug.println("Temps de récupération des données du lidar : " + String(elapsed) + "ms");
    start_millis = millis();

    // SerialDebug.println("First lidarPoint(0): " + points[0].toString());
    // SerialDebug.println("Last lidarPoint(" + String(points.size()) + "): " + points[points.size() - 1].toString());
  }
  else { // pour tester sans lidar
    const char* input = "(31205,445);(31281,446);(31357,446);(31433,447);(31509,448);(31585,449);(31661,450);(31737,451);(31813,452);(31889,454);(31965,455);(32041,456);(27397,244);(27469,506);(27541,506);(27613,506);(27685,508);(27757,509);(27829,507);(27901,504);(27973,501);(28045,498);(28117,496);(28189,493);(28262,491);(28333,488);(28404,485);(28475,483);(28546,480);(28617,478);(28688,475);(28759,473);(28830,471);(28901,469);(28972,467);(29043,466);(29122,464);(29193,463);(29264,461);(29335,460);(29406,458);(29477,456);(29548,454);(29619,453);(29690,452);(29761,451);(29832,450);(29903,449);(29977,448);(30048,447);(30119,446);(30190,445);(30261,444);(30332,444);(30403,444);(30474,444);(30545,444);(30616,444);(30687,444);(30758,444);(30837,444);(30908,444);(30979,444);(31050,444);(31121,445);(31192,445);(31263,446);(31334,446);(31405,447);(31476,448);(31547,449);(31618,450);(31692,451);(31765,452);(31838,453);(31911,455);(31984,456);(32057,457);(32130,458);(32203,460);(32276,461);(32349,463);(32422,465);(32495,467);(32569,469);(32640,471);(32711,473);(32782,475);(32853,477);(32924,479);(32995,481);(33066,484);(33137,487);(33208,489);(33279,492);(33350,495);(33428,498);(33499,501);(33570,505);(33641,508);(33712,510);(33783,512);(33854,514);(33925,516);(33996,523);(34067,528);(34138,534);(34209,540);(34286,545);(34362,551);(34438,556);(34514,561);(34590,567);(34666,574);(34742,580);(34818,588);(34894,595);(34970,602);(35046,608);(35122,614);(35201,619);(639,591);(703,596);(767,596);(844,595);(916,592);(988,585);(1060,583);(1132,579);(1204,575);(1276,572);(1348,569);(1420,566);(1492,563);(1564,560);(1636,558);(1757,555);(1828,553);(1899,551);(1970,548);(2041,546);(2112,544);(2183,542);(2254,540);(2325,538);(2396,536);(2467,535);(2538,533);(2616,532);(2687,530);(2758,529);(2829,527);(2900,526);(2971,525);(3042,524);(3113,523);(3184,523);(3255,522);(3326,521);(3397,521);(3468,520);(3539,520);(3610,519);(3681,518);(3752,518);(3823,517);(3894,517);(3965,517);(4036,517);(4107,517);(4178,517);(4249,518);(4331,518);(4397,518);(4463,518);(4529,519);(4595,519);(4661,520);(4727,521);(4793,522);(4859,523);(4925,524);(4991,525);(5057,527);(5137,528);(5208,530);(5279,532);(5350,534);(5421,536);(5492,538);(5563,540);(5634,542);(5705,544);(5776,546);(5847,549);(5918,551);(5992,554);(6063,556);(6134,559);(6205,562);(6276,565);(6347,568);(6418,571);(6489,575);(6560,579);(6631,583);(6702,587);(6773,591);(6851,598);(6922,603);(6993,608);(7064,610);(7135,617);(7206,622);(7277,627);(7348,632);(7419,638);(7490,644);(7561,650);(7632,656);(7710,663);(7781,670);(7852,676);(7923,683);(7994,689);(8065,697);(8136,704);(8207,712);(8278,720);(8349,728);(8420,737);(8491,746);(8566,757);(8637,768);(8708,779);(8779,789);(8850,800);(8921,813);(8992,826);(9063,840);(9134,854);(9205,867);(9276,881);(9347,896);(9422,912);(9493,930);(9564,950);(9635,969);(9706,990);(9777,1011);(9848,1033);(9919,1055);(9990,1078);(10061,1103);(10132,1129);(10203,1157);(10281,1193);(10358,1221);(10435,1257);(10512,1290);(10589,1328);(10666,1374);(10743,1417);(10820,1462);(10897,1515);(10974,1579);(11051,1633);(11128,1698);(11205,1770);(11276,1845);(11347,1956);(11418,2058);(11489,2049);(11560,2044);(11631,2042);(11702,421);(11773,389);(11844,380);(11915,378);(11986,374);(12063,371);(12134,368);(12205,365);(12276,363);(12347,361);(12418,361);(12489,363);(12560,365);(12631,369);(12702,376);(12773,382);(12844,389);(12926,396);(12997,405);(13068,417);(13139,431);(13210,1988);(13281,1990);(13352,1994);(13423,2001);(13494,2006);(13565,2008);(13636,2011);(13707,2014);(13779,2017);(13850,2019);(13921,2023);(13992,2027);(14063,2034);(14134,2040);(14205,2045);(14276,2051);(14347,2052);(14418,2055);(14489,1004);(14560,998);(14630,996);(14701,994);(14772,995);(14843,1001);(14914,1008);(14985,1019);(15056,1027);(15127,1026);(15198,2173);(15269,2189);(15340,2207);(15411,2220);(15493,2234);(15564,2248);(15635,2261);(15706,2278);(15777,2298);(15848,2315);(15919,2332);(15990,2350);(16061,2370);(16132,2383);(16203,2351);(16274,2315);(16349,2270);(16425,2229);(16501,2197);(16577,2166);(16653,2134);(16729,2104);(16805,2068);(16881,2042);(16957,2017);(17033,1992);(17109,1967);(17185,1942);(17258,1918);(17329,1894);(17400,1872);(17471,1856);(18413,1607);(18487,1605);(18561,1597);(18635,1585);(18709,1572);(18783,1559);(18857,1547);(18931,1536);(19009,1526);(19078,1516);(19147,1506);(19216,1162);(19285,1160);(19354,1158);(19423,1127);(19492,1050);(19561,1057);(19630,1074);(19699,1087);(19768,1104);(19850,1183);(19921,1224);(19992,1272);(20063,1408);(20134,1402);(20205,1397);(20276,1392);(20347,1387);(20418,1382);(20489,1377);(20560,1372);(20631,1370);(20709,1366);(20773,1362);(20837,1359);(20901,1356);(20965,1353);(21029,1351);(21093,1348);(21157,1346);(21221,1344);(21285,1342);(21349,1340);(21413,1338);(21490,1336);(21561,1335);(21632,1334);(21703,1334);(21774,1333);(21845,1332);(21916,1332);(21987,1332);(22058,1332);(22129,1332);(22200,1333);(22271,1334);(22349,1336);(22419,1337);(22489,1339);(22559,1341);(22629,1343);(22699,1345);(22769,1347);(22839,1349);(22909,1352);(22979,1355);(23049,1359);(23119,1362);(23190,1366);(23261,1373);(23332,1378);(23403,1380);(23474,1384);(23545,1386);(23616,1387);(23687,1318);(23758,1276);(23829,1236);(23900,1199);(23971,1164);(24049,1128);(24121,1100);(24193,1072);(24265,1044);(24337,1018);(24409,995);(24481,969);(24553,946);(24625,923);(24697,900);(24769,881);(24841,861);(24919,843);(24990,828);(25061,812);(25132,795);(25203,780);(25274,767);(25345,754);(25416,741);(25487,728);(25558,716);(25629,704);(25700,691);(25722,675);(25791,643);(25860,644);(25929,645);(25998,643);(26067,636);(26136,626);(26205,250);(26274,250);(26343,250);(26412,252);(26481,583);(26562,578);(26637,576);(26712,574);(26787,568);(26862,563);(26937,557);(27012,551);(27087,544);(27162,535);(27237,245);(27312,244);(27387,246);(27468,259);(27540,503);(27612,505);(27684,508);(27756,508);(27828,507);(27900,504);(27972,501);(28044,498);(28116,496);(28188,493);(28260,490);(28334,488);(28406,485);(28478,483);(28550,481);(28622,479);(28694,476);(28766,474);(28838,472);(28910,470);(28982,468);(29054,466);(29126,464);(29201,462);(29272,461);(29343,459);(29414,457);(29485,456);(29556,454);(29627,453);(29698,452);(29769,450);(29840,449);(29911,448);(29982,447);(30063,446);(30134,445);(30205,445);(30276,445);(30347,444);(30418,444);(30489,444);(30560,444);(30631,444);(30702,444);(30773,444);(30844,444);(30923,444);(30994,444);(31065,445);(31136,445);(31207,445);(31278,446);(31349,446);(31420,447);(31491,448);(31562,449);(31633,450);(31704,451);(31779,452);(31850,453);(31921,454);(31992,455);(32063,456);(32134,458);(32205,460);(32276,461);(32347,463);(32418,465);(32489,467);(32560,468);(32637,470);(32708,472);(32779,474);(32850,477);(32921,479);(32992,481);(33063,484);(33134,486);(33205,489);(33276,491);(33347,494);(33418,497);(33496,500);(33567,503);(33638,506);(33709,509);(33780,512);(33851,515);(33922,517);(33993,521);(34064,528);(34135,534);(34206,540);(34277,545);(34351,551);(34422,556);(34493,561);(34564,566);(34635,572);(34706,578);(34777,585);(34848,592);(34919,598);(34990,605);(35061,612);(35132,614)";
    const char *p = input;
    float distance, angle;
    while (*p) { 
      if (sscanf(p, "(%f,%f)", &angle, &distance) == 2) {
        LidarPoint lidarPoint(distance, 1, angle);

        if(lidarPoint.distance() > 13) { // on ne prend pas les points < 13cm
          points.push_back(lidarPoint); 
          if(lidarPoint.distance() > distance_max) {
            distance_max = lidarPoint.distance();
          }        
        }

        if(points.size() > 700) { // si trop de données: plantage
          break;
        }

        while (*p && *p != ';') p++; 
        if (*p == ';') p++; 
      } else {
            break; // Si la lecture échoue, sortir de la boucle
      }
    }

    // const char* input = "(199.93,456.10);(195.82,462.23);(191.60,468.32);(188.35,477.17);(184.20,484.14);(179.91,491.08);(175.47,497.99);(171.21,505.81);(166.78,513.60);(162.48,522.31);(157.71,530.04);(153.04,538.68);(116.87,-16.32);(106.15,-13.48);(104.32,-11.94);(101.47,-10.34);(102.88,5.03);(104.80,6.43);(112.69,8.32);(119.56,10.31);(126.39,12.48);(127.21,14.16);(467.43,57.89);(467.67,63.81);(468.82,69.90);(467.96,75.46);(465.01,80.91);(461.03,86.11);(456.98,91.23);(453.86,96.47);(449.70,101.43);(446.47,106.53);(443.17,111.56);(439.82,116.53);(436.42,121.44);(432.96,126.28);(429.45,131.05);(425.55,136.79);(421.93,141.42);(419.20,146.31);(415.48,150.81);(411.70,155.24);(407.89,159.60);(404.95,164.27);(401.96,168.89);(398.00,173.06);(394.92,177.57);(391.78,182.02);(388.59,186.43);(385.12,191.26);(380.98,195.04);(376.80,198.75);(371.70,201.90);(366.59,204.97);(360.59,207.43);(350.29,207.24);(340.04,206.83);(252.69,157.96);(228.87,146.99);(327.22,215.85);(332.84,225.43);(331.34,231.15);(330.08,236.40);(327.93,241.06);(324.92,245.11);(322.64,249.73);(320.31,254.33);(317.91,258.91);(315.44,263.47);(313.67,268.66);(311.07,273.19);(309.15,278.36);(306.41,282.85);(302.38,287.15);(299.62,291.47);(296.79,295.76);(293.91,300.03);(290.28,303.54);(287.28,307.75);(284.90,312.67);(282.45,317.57);(280.57,323.21);(277.95,328.09);(275.25,332.96);(272.48,337.81);(268.41,342.32);(265.36,347.21);(262.23,352.07);(259.03,356.91);(255.74,361.73);(252.37,366.52);(248.92,371.28);(245.94,376.84);(242.33,381.55);(239.16,387.08);(235.89,392.58);(232.52,398.07);(228.56,403.81);(224.91,409.27);(221.63,415.60);(217.77,421.01);(214.25,427.29);(210.61,433.55);(206.85,439.78);(204.21,448.72);(199.36,453.08);(195.22,459.23);(192.09,468.12);(188.03,475.15);(182.98,482.47);(178.71,489.39);(174.28,496.29);(169.72,503.15);(165.32,510.92);(160.75,518.66);(156.02,526.36);(151.39,534.99);(146.57,543.59);(141.31,551.17);(136.13,559.68);(130.75,568.15);(125.21,578.61);(119.72,587.93);(114.03,597.21);(108.31,607.42);(102.19,616.59);(96.31,628.67);(90.30,641.68);(83.72,652.65);(75.86,654.62);(67.85,655.50);(59.02,647.31);(-89.77,527.41);(-96.12,525.28);(-102.05,521.10);(-107.07,512.95);(-112.98,509.63);(-118.60,505.27);(-124.50,499.72);(-129.58,495.33);(-134.83,491.85);(-140.02,488.32);(-145.15,484.73);(-149.91,480.14);(-154.59,475.50);(-159.18,470.82);(-164.01,467.04);(-168.78,463.21);(-173.83,460.27);(-178.47,456.34);(-184.19,454.06);(-189.42,450.82);(-194.59,447.53);(-199.71,444.17);(-204.36,439.85);(-209.36,436.38);(-213.86,431.96);(-218.74,428.38);(-223.57,424.76);(-228.82,421.95);(-234.51,419.96);(-239.70,417.02);(-245.57,413.59);(-251.63,411.10);(-257.66,408.53);(-262.59,404.19);(-266.33,398.13);(-269.95,392.05);(-271.15,382.67);(-271.55,372.53);(-272.34,363.25);(-276.58,358.76);(-280.14,353.45);(-284.23,348.87);(-288.71,345.17);(-292.91,341.62);(-297.72,338.77);(-302.51,335.85);(-307.26,332.86);(-312.68,330.54);(-318.09,328.13);(-324.19,326.35);(-329.58,323.76);(-334.95,321.10);(-340.31,318.35);(-345.65,315.51);(-350.28,311.87);(-354.87,308.16);(-359.43,304.39);(-363.94,300.55);(-369.20,297.27);(-374.43,293.91);(-379.63,290.46);(-384.80,286.93);(-389.95,283.31);(-395.06,279.61);(-400.14,275.83);(-405.19,271.97);(-409.55,267.19);(-413.67,262.63);(-417.75,258.01);(-420.91,252.81);(-424.01,247.58);(-428.79,243.29);(-435.27,239.89);(-440.86,235.89);(-447.30,232.25);(-453.71,228.49);(-459.21,224.17);(-464.66,219.74);(-470.49,214.31);(-475.85,209.68);(-481.17,204.94);(-485.53,199.72);(-491.68,195.17);(-497.80,190.49);(-503.87,185.69);(-509.91,180.77);(-515.89,175.72);(-524.68,171.49);(-531.54,166.47);(-538.35,161.31);(-546.16,155.99);(-554.00,150.21);(-561.78,144.24);(-569.50,138.09);(-577.15,131.76);(-585.71,125.46);(-594.21,118.95);(-603.62,112.42);(-611.98,105.49);(-621.24,98.51);(-631.41,91.44);(-640.52,83.98);(-648.49,76.64);(-655.35,69.23);(-664.13,61.84);(-674.81,54.41);(-818.66,-46.77);(-830.97,-58.11);(-844.12,-69.84);(-860.08,-82.21);(-878.85,-95.32);(-894.44,-108.56);(-911.81,-122.47);(-931.94,-137.28);(-952.83,-152.79);(-973.47,-168.85);(-1007.44,-188.90);(-1007.20,-200.71);(-1006.81,-212.54);(-1003.36,-223.73);(-996.86,-234.18);(-986.38,-243.56);(-977.74,-253.23);(-969.98,-262.99);(-962.12,-272.62);(-953.21,-281.81);(-945.18,-291.14);(-937.05,-300.31);(-928.41,-310.64);(-922.60,-321.46);(-914.78,-331.51);(-906.86,-341.41);(-898.84,-351.17);(-890.71,-360.77);(-882.48,-370.24);(-875.07,-379.95);(-867.56,-389.53);(-859.95,-398.99);(-852.23,-408.32);(-845.31,-417.96);(-836.87,-428.07);(-828.85,-437.00);(-821.62,-446.29);(-814.28,-455.46);(-806.84,-464.51);(-800.16,-473.97);(-792.52,-482.81);(-785.63,-492.06);(-777.79,-500.67);(-770.69,-509.72);(-763.49,-518.67);(-756.18,-527.52);(-749.21,-537.37);(-741.69,-546.02);(-734.87,-555.17);(-727.93,-564.23);(-720.88,-573.21);(-713.72,-582.10);(-706.46,-590.90);(-699.08,-599.61);(-692.35,-608.88);(-685.49,-618.09);(-678.51,-627.21);(-671.42,-636.26);(-663.86,-645.57);(-657.34,-655.05);(-649.99,-663.75);(-643.21,-673.08);(-636.30,-682.35);(-629.27,-691.56);(-622.10,-700.69);(-614.80,-709.75);(-608.02,-719.50);(-601.09,-729.19);(-593.39,-738.03);(-585.57,-746.80);(-577.56,-756.79);(-570.53,-767.10);(-562.74,-776.54);(-555.38,-786.72);(-547.85,-796.83);(-541.83,-809.38);(-532.85,-817.71);(-524.82,-827.62);(-518.20,-840.02);(-510.32,-850.67);(-502.26,-861.24);(-494.02,-871.75);(-484.99,-883.66);(-476.36,-894.01);(-468.00,-905.17);(-459.43,-916.27);(-451.10,-928.19);(-442.55,-940.04);(-433.77,-951.82);(-424.77,-963.53);(-415.54,-975.16);(-406.09,-986.70);(-396.78,-999.09);(-387.23,-1011.41);(-377.09,-1022.69);(-366.88,-1034.89);(-356.75,-1047.94);(-346.03,-1059.95);(-335.37,-1072.80);(-324.73,-1086.51);(-313.79,-1100.12);(-302.83,-1114.59);(-291.80,-1129.93);(-280.67,-1146.13);(-269.40,-1163.21);(-257.75,-1180.18);(-244.89,-1196.19);(-232.69,-1212.88);(-219.93,-1228.47);(-206.82,-1243.92);(-193.37,-1259.24);(-179.42,-1273.42);(-164.39,-1281.50);(-89.31,-771.85);(-79.53,-770.91);(-69.61,-767.85);(-59.78,-764.67);(-50.10,-762.36);(-39.51,-758.97);(-29.96,-752.40);(-20.62,-747.72);(-11.38,-740.91);(-2.31,-734.00);(6.61,-727.97);(15.46,-725.84);(24.19,-721.59);(32.91,-719.25);(41.64,-717.79);(50.55,-719.23);(59.58,-721.54);(69.40,-728.70);(79.06,-733.75);(89.22,-741.65);(99.46,-748.42);(109.45,-752.08);(118.92,-751.65);(128.73,-753.08);(138.59,-754.38);(148.31,-754.56);(158.88,-758.54);(170.44,-766.27);(330.97,-1405.56);(348.61,-1396.13);(364.84,-1385.78);(381.14,-1376.20);(397.24,-1366.43);(414.04,-1359.34);(430.11,-1350.15);(446.01,-1340.76);(461.75,-1331.19);(477.31,-1321.44);(491.65,-1308.70);(507.55,-1300.46);(522.58,-1290.18);(538.93,-1280.19);(553.95,-1271.58);(568.43,-1261.88);(582.76,-1252.02);(596.93,-1242.00);(610.94,-1231.82);(624.34,-1220.59);(637.55,-1209.22);(651.53,-1199.47);(665.86,-1190.43);(679.56,-1180.36);(693.62,-1170.99);(708.17,-1161.08);(722.24,-1150.01);(735.60,-1137.94);(749.33,-1126.55);(762.88,-1114.99);(776.26,-1103.28);(790.05,-1092.21);(804.28,-1081.78);(818.37,-1071.16);(832.32,-1060.35);(845.50,-1048.59);(859.15,-1037.43);(873.20,-1025.64);(886.44,-1016.87);(899.61,-1007.92);(912.68,-998.81);(927.03,-990.99);(940.63,-982.25);(954.15,-973.33);(967.59,-964.22);(980.23,-954.24);(993.50,-944.78);(1007.41,-935.82);(1021.25,-926.66);(1038.05,-918.39);(1050.71,-906.95);(1067.83,-899.20);(1080.28,-887.35);(1096.50,-878.47);(1111.10,-868.08);(1126.39,-858.08);(1111.90,-825.77);(1045.76,-757.01);(1002.64,-707.28);(959.28,-659.30);(918.24,-614.71);(903.76,-585.79);(877.12,-553.21);(852.37,-522.94);(750.57,-447.79);(732.71,-424.91);(710.91,-400.58);(702.66,-384.54);(695.01,-369.23);(686.20,-353.72);(678.90,-339.38);(670.45,-324.83);(662.66,-310.98);(656.11,-296.52);(649.65,-283.96);(642.99,-271.61);(635.19,-259.09);(626.26,-246.44);(617.11,-234.05);(610.57,-222.95);(603.85,-212.06);(598.85,-202.00);(594.65,-192.41);(590.30,-182.95);(585.81,-173.64);(581.27,-164.15);(575.53,-154.86);(570.62,-145.98);(566.56,-137.48);(562.37,-129.11);(558.06,-120.86);(553.64,-112.74);(549.10,-104.75);(544.45,-96.88);(539.69,-89.15);(533.83,-81.40);(115.87,-16.20);(103.17,-13.11);(101.34,-11.60);(100.88,4.93);(104.80,6.43);(112.69,8.32);(119.56,10.31);(126.39,12.48);(127.21,14.16);(466.44,57.77);(470.64,64.22);(469.81,70.05);(468.90,75.86);(464.97,81.15);(460.98,86.35);(456.93,91.47);(453.81,96.71);(449.65,101.66);(446.41,106.76);(443.11,111.80);(439.76,116.77);(436.36,121.67);(432.89,126.51);(429.38,131.27);(425.82,135.98);(422.00,141.20);(419.28,146.09);(415.55,150.59);(411.79,155.03);(408.90,159.75);(405.04,164.06);(402.05,168.68);(399.01,173.25);(395.01,177.36);(391.87,181.82);(388.69,186.23);(385.46,190.59);(381.08,194.84);(376.87,198.61);(371.74,201.84);(366.59,204.97);(357.96,206.00);(350.21,207.36);(339.08,206.49);(264.45,165.57);(229.58,147.73);(324.49,214.53);(332.57,225.84);(333.84,232.81);(331.01,236.81);(328.01,240.95);(325.76,245.65);(322.64,249.73);(320.26,254.38);(317.82,259.02);(315.30,263.64);(313.49,268.88);(311.58,274.12);(308.86,278.68);(306.06,283.22);(303.21,287.73);(299.52,291.57);(296.69,295.86);(293.81,300.13);(290.86,304.37);(287.86,308.58);(285.47,313.50);(283.00,318.41);(279.80,322.55);(277.19,327.43);(274.50,332.28);(271.73,337.12);(268.89,341.94);(265.54,347.07);(262.42,351.93);(259.21,356.78);(255.93,361.59);(252.56,366.38);(249.68,371.98);(246.14,376.72);(243.06,382.27);(239.89,387.80);(236.09,392.46);(232.73,397.94);(229.26,403.41);(225.26,409.08);(221.99,415.40);(218.13,420.82);(214.63,427.11);(210.99,433.37);(208.51,442.31);(203.77,446.72);(199.75,452.91);(195.62,459.06);(192.49,467.95);(188.44,474.98);(184.25,481.99);(179.90,488.95);(175.41,495.89);(170.77,502.79);(166.30,510.60);(161.66,518.38);(156.84,526.12);(152.13,534.78);(146.97,542.44);(141.89,551.02);(136.85,560.54);(131.60,570.01);(125.91,578.45);(119.93,587.89);(114.32,598.17);(108.48,608.40);(102.41,618.58);(96.24,629.69);(90.24,643.71);(83.39,653.70);(75.17,653.69);(66.76,651.59);(58.32,648.38)";
    // const char *p = input;
    // float x, y;
    // while (*p) { 
    //     if (sscanf(p, "(%f,%f)", &x, &y) == 2) {
    //         points_cart.push_back(Vector2(x, y));
    //         while (*p && *p != ';') p++; 
    //         if (*p == ';') p++; 
    //     } else {
    //         break; // Si la lecture échoue, sortir de la boucle
    //     }

    //     if(points_cart.size() > 700) { // si trop de données: plantage
    //       break;
    //     }
    // }
  }

  start_millis = millis();

  if(show_log) {
    full_log += "** LIDAR points: ";
    for (size_t i = 0; i < points.size(); i++) {
      LidarPoint lidarPoint = points[i];
      full_log += "(" + String(lidarPoint.angle()) + "," + String(lidarPoint.distance()) + ");" ;
    }
    if (full_log.length() > 0 && full_log[full_log.length() - 1] == ';') { full_log = full_log.substring(0, full_log.length() - 1); }
    full_log += "\r\n";
  }

  // conersion en coordonnées cartésiennes :
  if(!points.empty()) {
    for (size_t i = 0; i < points.size(); i++) {
      LidarPoint lidarPoint = points[i];
      float x = lidarPoint.distance() * std::sin(lidarPoint.angle() / 18000.0 * M_PI);
      float y = lidarPoint.distance() * std::cos(lidarPoint.angle() / 18000.0 * M_PI);
      points_cart.push_back(Vector2(x, y));
    }
  }

  if(show_log) {
    full_log += "** CARTESIAN points: ";
    for (size_t i = 0; i < points_cart.size(); i++) {
      Vector2 point = points_cart[i];
      full_log += "(" + String(point.x()) + "," + String(point.y()) + ");";
    }
    if (full_log.length() > 0 && full_log[full_log.length() - 1] == ';') { full_log = full_log.substring(0, full_log.length() - 1); }
    full_log += "\r\n";
  }
   
  int numRho = distance_max;
  int numTheta = 180;
  double thetaStep = 1 * M_PI / 180.0;
  int threshold = 30;

  // Test avec 5 points simples :
  // int numRho = 4;
  // points_cart.clear();
  // points_cart.push_back(Vector2(0, -4));
  // points_cart.push_back(Vector2(1, -3));
  // points_cart.push_back(Vector2(2, -2));
  // points_cart.push_back(Vector2(3, -1));
  // points_cart.push_back(Vector2(4, 0));
  
  start_millis = millis();
  std::vector<Line> lines = houghTransform(points_cart, numRho, numTheta, thetaStep, threshold);
  unsigned long elapsed = millis() - start_millis;
  // SerialDebug.println("Temps hough transform : " + String(elapsed) + "ms");
    
  // Tri des lignes selon nb_accumulators du plus grand au plus petit
  std::sort(lines.begin(), lines.end(), [](const Line& a, const Line& b) {
      return a.nb_accumulators > b.nb_accumulators;
  });

  std::vector<Line> walls;
  if(lines.empty()) {
    return infos;
  }

  // SerialDebug.println("lines: " + String(lines.size()));
  
  for (const auto& line : lines) {
    bool isDifferentEnough = true;
    bool isParallelOrPerpendicular = false;
    String log_line = "";

    if(!walls.empty()) {
      for (const auto& wall : walls) {
          double angleDifference = std::abs(line.theta - wall.theta);
          
          if (std::abs(line.rho - wall.rho) < rhoTolerance && angleDifference < thetaMargin) {
              isDifferentEnough = false;
          }

          double angleDifferenceWithFirstWall = std::abs(line.theta - walls[0].theta);
          if (std::abs(angleDifference - M_PI / 2) < thetaTolerancePerpendiculaire) {
              // log_line += ", PERPENDICULAIRE angleDifferenceWithFirstWall=" + String(angleDifferenceWithFirstWall);
              isParallelOrPerpendicular = true; // perpendiculaire
          }

          if (angleDifference < thetaToleranceParallel
                && abs(wall.rho - line.rho) > 0.9 * LARGEUR_TERRAIN
                && ((wall.rho > 0 && line.rho < 0) || (wall.rho < 0 && line.rho > 0))) {
              // log_line += ", PARALLEL angleDifference=" + String(angleDifference) + 
                  // ", abs(wall.rho - line.rho)=" + String(abs(wall.rho - line.rho)));
              isParallelOrPerpendicular = true; // parallèle
          }
      }
    }

    if (walls.empty() || (isDifferentEnough && isParallelOrPerpendicular)) {
        // SerialDebug.println("ligne ajoutée -> rho: " + String(line.rho) + ", theta: " + String(line.theta) + ", accumulators: " 
          // + String(line.nb_accumulators) + log_line);
        walls.push_back(line);
    }

    if(walls.size() == 4) {
      break;
    }
  }

  std::vector<double> rhos;
  std::vector<double> thetas;
  
  // Afficher les lignes détectées
  // SerialDebug.println("lines: " + String(lines.size()));
  // for (const auto& line : lines) {
      // SerialDebug.println("line -> rho: " + String(line.rho) + ", theta: " + String(line.theta) + ", accumulators: " 
          // + String(line.nb_accumulators));
      // rhos.push_back(line.rho);
      // thetas.push_back(line.theta);
  // }

  for (const auto& wall : walls) {
    // SerialDebug.println("wall -> rho: " + String(wall.rho) + ", theta: " + String(wall.theta) + ", accumulators: " 
        // + String(wall.nb_accumulators));
    rhos.push_back(wall.rho);
    thetas.push_back(wall.theta);
  }

  // Afficher les murs détectés
  if(show_log) {
    full_log += "** Walls rho: ";
    for (size_t i = 0; i < rhos.size(); i++) {
      full_log += String(rhos[i]) + ","; // * 10
    }
    if (full_log.length() > 0 && full_log[full_log.length() - 1] == ',') { full_log = full_log.substring(0, full_log.length() - 1); }
    full_log += "\r\n";

    full_log += "** Walls theta: ";
    for (size_t i = 0; i < thetas.size(); i++) {
      full_log += String(thetas[i]) + ",";
    }
    if (full_log.length() > 0 && full_log[full_log.length() - 1] == ',') { full_log = full_log.substring(0, full_log.length() - 1); }
    full_log += "\r\n";
  }

  // SerialDebug.println("*** walls: " + String(walls.size()));
  // for (const auto& wall : walls) {
  //     SerialDebug.println("wall -> rho: " + String(wall.rho) + ", theta: " + String(wall.theta) + ", accumulators: " 
  //         + String(wall.nb_accumulators));
  //     rhos.push_back(wall.rho);
  //     thetas.push_back(wall.theta);
  // }


  // Trouver les coins
  std::vector<Vector3> corners;
  int limit_coord = 5000;
  for (size_t i = 0; i < walls.size(); i++) {
    for (size_t j = i+1; j < walls.size(); j++) {      
      double x, y;
      if (findIntersection(walls[i], walls[j], x, y)) {
          // SerialDebug.println("Intersection entre le mur " + String(i) + " et le mur " + String(j) + ": x = " 
              // + String(x) + ", y = " + String(y));
          if(std::abs(x) < limit_coord && std::abs(y) < limit_coord && corners.size() < 4) {
            Vector3 v ;
            v.x = x;
            v.y = y;
            corners.push_back(v);
          }
      }
    }
  }

  // mur le plus proche
  double nearest = 100000;
  for (size_t i = 0; i < walls.size(); i++) {
    if(abs(walls[i].rho) < nearest) {
      nearest = walls[i].rho;
    }
  }
  infos.nearestWall_distance = abs(nearest);

  if(corners.size() != 4) {
    if(show_log) {
      full_log += "****************"; 
      SerialDebug.println(full_log);    
    }
    return infos;
  }

  Vector3 centroid = computeCentroid(corners);
  // SerialDebug.println("Centre du terrain : (" + String(centroid.x) + ", " + String(centroid.y) + ")");

  sortPointsClockwise(corners, centroid);
  
  if(show_log) {
    full_log += "** corners x: ";
    for (size_t i = 0; i < corners.size(); i++) {
      full_log += String(corners[i].x) + ",";
    }
    if (full_log.length() > 0 && full_log[full_log.length() - 1] == ',') { full_log = full_log.substring(0, full_log.length() - 1); }
    full_log += "\r\n";

    full_log += "** corners y: ";
    for (size_t i = 0; i < corners.size(); i++) {
      full_log += String(corners[i].y) + ",";
    }
    if (full_log.length() > 0 && full_log[full_log.length() - 1] == ',') { full_log = full_log.substring(0, full_log.length() - 1); }
    full_log += "\r\n";
  }
  

  if(show_log) {
    full_log += "** centroid: " + String(centroid.x) + "," + String(centroid.y) + "\r\n";
  }

  // orientation (angle avec le côté du terrain le plus long)
  double maxDistance = 0;
  int firstCornerIndex = -1;
  int secondCornerIndex = -1;

  // trouver le mur le plus long 
  for (size_t i = 0; i < corners.size(); i++) {
    // SerialDebug.println("corner " + String(i) + ": x=" + String(corners[i].x) + ", y=" + String(corners[i].y));
    size_t nextIndex = (i + 1) % corners.size();
    double distanceBetweenCorners = distance3(corners[i], corners[nextIndex]);
    // SerialDebug.println("distanceBetweenCorners: " + String(distanceBetweenCorners));

    double deltaY = corners[nextIndex].y - corners[i].y;
    double deltaX = corners[nextIndex].x - corners[i].x;
    double angleRadians = std::atan2(deltaY, deltaX);
    // SerialDebug.println("orientation: " + String(angleRadians * (180.0 / M_PI)) + "°"); 

    if (distanceBetweenCorners > maxDistance) {
      maxDistance = distanceBetweenCorners;
      firstCornerIndex = i;
      secondCornerIndex = (i + 1) % corners.size();      
    }
  }  

  // on a trouvé le mur le plus long 
  if(firstCornerIndex != -1 && secondCornerIndex != -1) {
    double deltaY = corners[secondCornerIndex].y - corners[firstCornerIndex].y;
    double deltaX = corners[secondCornerIndex].x - corners[firstCornerIndex].x;
    double angleRadians = std::atan2(deltaY, deltaX); // renvoie un angle entre -PI et +PI
    
    // std::pair<double, double> newCoordinates = convertCoordinates(0, 0, centroid.x, centroid.y, angleRadians);
    // Vector3 center;
    // center.x = newCoordinates.first;
    // center.y = newCoordinates.second;
    infos.orientation = angleRadians; //  + M_PI
    // SerialDebug.println("mur le plus long entre corner " + String(firstCornerIndex + 1) + " et corner " + String(secondCornerIndex + 1));
    // SerialDebug.println("orientation: " + String(angleRadians * (180.0 / M_PI)) + "°");
  }

/*
  Vector3 O = { 0, 0 }; // Coordonnées du point O dans le repère (x, y)
  double theta = infos.orientation; // Inclinaison theta

  // Calculer et afficher les coordonnées de O dans le repère (u, v)
  Vector3 O_uv = transformToUV(centroid, O, theta);
  SerialDebug.println("Coordonnées de O dans le repère (u, v): (" + String(O_uv.x) + ", " + String(O_uv.y) + ")");
  infos.coordinates = O_uv;
*/
  // à quel mur le robot fait face ?
  if(walls.size() >= 2) {
    std::vector<std::pair<double, const Line*>> xWallPairs;

    // on calcule en quel x chaque wall coupe l'axe des abscisses. La plus petite valeur négative est le mur le plus proche
    // de l'avant du robot
    for (const auto& wall : walls) {
        double x = calculateX(wall);
        xWallPairs.push_back({x, &wall});
        //SerialDebug.println("wall rho: " + String(wall.rho) + ", theta: " + String(wall.theta) + ", croisement x = " + String(x));
    }
    std::sort(xWallPairs.begin(), xWallPairs.end());
    
    double xFacing = xWallPairs[1].first;
    const Line& frontWall = *xWallPairs[1].second;
    infos.frontWall_distance = -xFacing;
    // SerialDebug.println("xFacing: " + String(xFacing) + ", f: " + String(frontWall.rho));

    if (xWallPairs.size() == 4) {
      double xSecond = xWallPairs[0].first;
      const Line& secondWall = *xWallPairs[0].second;

      double angleDifference = secondWall.theta - frontWall.theta;
      angleDifference = fmod(angleDifference + 2 * M_PI, 2 * M_PI);
      bool murDroite = false;
      if (angleDifference < M_PI) {
        // secondWall est le mur de droite
        murDroite = true;
      }

      // mur arrière
      const Line* rearWallPtr = nullptr;
      for (const auto& pair : xWallPairs) {
        if (pair.second != &frontWall && std::abs(pair.second->theta - frontWall.theta) < 0.5) { // Utiliser une petite tolérance pour comparer les angles
            rearWallPtr = pair.second;
            break;
        }
      }

      if (rearWallPtr != nullptr) {
        // Trouvez le mur qui n'a pas encore été détecté
        const Line* lastWallPtr = nullptr;
        for (const auto& wall : walls) {
            if(&wall != &frontWall && &wall != rearWallPtr && &wall != &secondWall) {
              lastWallPtr = &wall;
            }
        }

        if(lastWallPtr != nullptr) {
          double longueur = abs(frontWall.rho) + abs(rearWallPtr->rho); // devrait être proche de 243 cm
          double largeur = abs(secondWall.rho) + abs(lastWallPtr->rho); // devrait être proche de 182 cm
          if(largeur > longueur) {
            largeur = abs(frontWall.rho) + abs(rearWallPtr->rho); 
            longueur = abs(secondWall.rho) + abs(lastWallPtr->rho);
          }


          // SerialDebug.println("longueur: " + String(longueur) + ", largeur: " + String(largeur));
          // SerialDebug.println("Front wall rho: " + String(frontWall.rho));
          // SerialDebug.println("Rear wall rho: " + String(rearWallPtr->rho));
          
          Vector3 coordinates;
          if(abs(frontWall.rho) > abs(rearWallPtr->rho)) { // y négatif
            coordinates.y = -(longueur / 2.0 - abs(rearWallPtr->rho));            
          }
          else {
            coordinates.y = longueur / 2.0 - abs(frontWall.rho);    
          }

          if(murDroite) { // secondWall est le mur de droite
            // SerialDebug.println("Right wall rho: " + String(secondWall.rho));
            // SerialDebug.println("Left wall rho: " + String(lastWallPtr->rho));

            if(abs(secondWall.rho) > abs(lastWallPtr->rho)) { // x négatif
              coordinates.x = -(largeur / 2.0 - abs(lastWallPtr->rho));            
            }
            else {
              coordinates.x = largeur / 2.0 - abs(secondWall.rho);    
            }
          }
          else {
            // SerialDebug.println("Right wall rho: " + String(lastWallPtr->rho));
            // SerialDebug.println("Left wall rho: " + String(secondWall.rho));

            if(abs(lastWallPtr->rho) > abs(secondWall.rho)) { // x négatif
              coordinates.x = -(largeur / 2.0 - abs(secondWall.rho));            
            }
            else {
              coordinates.x = largeur / 2.0 - abs(lastWallPtr->rho);    
            }
          }

          infos.coordinates = coordinates;
        }
      }
    }
  }

  if(show_log) {
    full_log += "****************"; 
    SerialDebug.println(full_log);    
  }
  
  return infos;
}

bool findIntersection(const Line& l1, const Line& l2, double& x, double& y) {
    double sinTheta1 = std::sin(l1.theta);
    double sinTheta2 = std::sin(l2.theta);
    double cosTheta1 = std::cos(l1.theta);
    double cosTheta2 = std::cos(l2.theta);
    double determinant = cosTheta1 * sinTheta2 - sinTheta1 * cosTheta2;

    // Si le déterminant est 0, les lignes sont parallèles ou coïncidentes
    if (std::abs(determinant) < 1e-10) {
        return false; 
    }

    x = (l1.rho * sinTheta2 - l2.rho * sinTheta1) / determinant;
    y = (l2.rho * cosTheta1 - l1.rho * cosTheta2) / determinant;

    return true; 
}
