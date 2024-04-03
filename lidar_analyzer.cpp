#include "lidar_analyzer.h"
#include "lidar.h"
/*

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

// Fonction pour ordonner les points dans le sens des aiguilles d'une montre
void sortPointsClockwise(std::vector<Vector3>& points) {
    Vector3 center = computeCentroid(points);

    std::sort(points.begin(), points.end(), [center](const Vector3& a, const Vector3& b) {
        double angleA = atan2(a.y - center.y, a.x - center.x);
        double angleB = atan2(b.y - center.y, b.x - center.x);
        return angleA > angleB;
    });
}

std::vector<Line> houghTransform(const std::vector<Vector2>& points, int numRho, int numTheta, double thetaStep, int threshold) {
    std::vector<Line> lines;
    double rhoStep = numRho * 2 * numTheta / 16000.0;

    std::vector<int> accumulator(16000, 0);
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

FieldInfos getFieldInfos(bool readFromLidar = true, bool show_log = false) {  
  std::vector<LidarPoint> points;
  double distance_max = 0;

  if(readFromLidar) {
    unsigned long start_millis = millis();

    CircularLidarPointsBuffer lidarPointsBuffer = CircularLidarPointsBuffer(456);
    
    const int nb_tours_lidar = 55;

    for (int i = 0; i < nb_tours_lidar; i++) {
        std::vector<LidarPoint> lidarPoints = lidarPointsBuffer.getPoints();
        for (size_t j = 0; j < lidarPoints.size(); j++) {
          LidarPoint lidarPoint = lidarPoints[j];
          if(lidarPoint.distance() > 100) { // on ne prend pas les points < 10cm
            points.push_back(lidarPoint); 
            if(lidarPoint.distance() > distance_max) {
              distance_max = lidarPoint.distance();
            }        
          }
        }
    }

    unsigned long elapsed = millis() - start_millis;
    SerialDebug.println("Temps d'exécution : " + String(elapsed) + "ms");
    SerialDebug.println("First lidarPoint(0): " + points[0].toString());
    SerialDebug.println("Last lidarPoint(" + String(points.size()) + "): " + points[points.size() - 1].toString());

    if(show_log) {
      SerialDebug.println("*** log pour visualisation ***");
      String log = "";
      for (size_t i = 0; i < points.size(); i++) {
        LidarPoint lidarPoint = points[i];
        log += "(" + String(lidarPoint.angle()) + "," + String(lidarPoint.distance()) + ")," ;
      }
      SerialDebug.println(log);    
    }
  }
  else { // pour tester sans lidar
    String input = "(30332,730),(30403,730),(30474,730),(30545,730),(30616,730),(30687,730),(30758,730),(30829,730),(30900,730),(30971,731),(31042,732),(31113,733),(28843,769),(28914,765),(28985,761),(29056,757),(29127,754),(29198,751),(29269,749),(29340,747),(29411,745),(29482,743),(29553,741),(29624,739),(29704,737),(29773,736),(29842,735),(29911,733),(29980,732),(30049,731),(30118,730),(30187,730),(30256,730),(30325,730),(30394,730),(30463,730),(30543,730),(30614,730),(30685,729),(30756,729),(30827,729),(30898,729),(30969,730),(31040,731),(31111,732),(31182,733),(31253,733),(31324,732),(31401,730),(31472,729),(31543,728),(31614,728),(31685,730),(31756,732),(31827,735),(31898,737),(31969,740),(32040,743),(32111,746),(32182,750),(32257,753),(32328,756),(32399,759),(32470,763),(32541,766),(32612,770),(32683,777),(32754,782),(32825,784),(32896,791),(32967,793),(33038,800),(33115,806),(33186,811),(33257,816),(33328,822),(33399,828),(33470,834),(33541,840),(33612,846),(33683,853),(33754,860),(33825,867),(33896,874),(33972,882),(34044,889),(34116,897),(34188,905),(34260,914),(34332,924),(34404,935),(34476,945),(34548,950),(34620,950),(34692,946),(34764,925),(34843,899),(34914,875),(34985,862),(35056,854),(35127,846),(35198,841),(35269,836),(35340,826),(35411,792),(35482,781),(35553,772),(35624,766),(35704,764),(35775,761),(35846,0),(35917,87),(35988,82),(59,81),(130,85),(201,87),(272,87),(343,83),(414,81),(485,132),(561,676),(632,669),(703,663),(774,658),(845,657),(916,656),(987,655),(1058,653),(1129,650),(1200,647),(1271,644),(1342,642),(1419,639),(1490,636),(1561,633),(1632,630),(1703,627),(1774,624),(1845,621),(1916,618),(1987,616),(2058,614),(2129,612),(2200,609),(2275,606),(2346,602),(2417,599),(2488,595),(2559,592),(2630,590),(2701,588),(2772,586),(2843,584),(2914,582),(2985,581),(3056,579),(3133,578),(3204,576),(3275,575),(3346,575),(3417,575),(3488,575),(3559,575),(3630,575),(3701,575),(3772,575),(3843,575),(3914,576),(3993,576),(4063,577),(4133,577),(4203,578),(4273,579),(4343,579),(4413,580),(4483,581),(4553,583),(4623,585),(4693,586),(4763,588),(4836,590),(4907,592),(4978,593),(5049,595),(5120,597),(5191,599),(5262,601),(5333,603),(5404,605),(5475,607),(5546,610),(5617,612),(5694,615),(5765,617),(5836,620),(5907,623),(5978,627),(6049,630),(6120,633),(6191,636),(6262,640),(6333,644),(6404,648),(6475,652),(6547,656),(6618,660),(6689,667),(6760,672),(6831,679),(6902,686),(6973,697),(7044,705),(7115,704),(7186,696),(7257,690),(7328,680),(7404,669),(7470,658),(7536,647),(7602,638),(7668,628),(7734,618),(7800,610),(7866,602),(7932,593),(7998,584),(8064,576),(8130,569),(8208,562),(8279,555),(8350,548),(8421,541),(8492,535),(8563,529),(8634,523),(8705,517),(8776,512),(8847,507),(8918,502),(8989,497),(9065,495),(9142,491),(9219,487),(9296,483),(9373,479),(9450,475),(9527,471),(9604,468),(9681,465),(9758,461),(9835,458),(9912,455),(9993,452),(10064,450),(10135,447),(10206,444),(10277,441),(10348,439),(10419,437),(10490,434),(10561,432),(10632,429),(10703,427),(10774,425),(10854,423),(10925,421),(10996,419),(11067,417),(11138,416),(11209,414),(11280,413),(11351,411),(11422,410),(11493,409),(11564,408),(11635,407),(11708,405),(11784,404),(11860,403),(11936,402),(12012,401),(12088,400),(12164,400),(12240,400),(12316,400),(12392,400),(12468,400),(12544,400),(12618,400),(12689,400),(12760,400),(12831,400),(12902,400),(12973,400),(13044,400),(13115,400),(13186,400),(13257,400),(13328,400),(13399,400),(13476,400),(13547,400),(13618,401),(13689,401),(13760,402),(13831,403),(13902,404),(13973,405),(14044,406),(14115,407),(14186,409),(14257,410),(14329,411),(14400,413),(14471,414),(14542,415),(14613,417),(14684,418),(14755,420),(14826,422),(14897,424),(14968,426),(15039,429),(15110,431),(15190,434),(15263,436),(15336,439),(15409,441),(15482,444),(15555,446),(15628,449),(15701,452),(15774,455),(15847,458),(15920,462),(15993,465),(16061,469),(16131,472),(16201,476),(16271,480),(16341,487),(16411,489),(16481,493),(16551,497),(16621,504),(16691,509),(16761,514),(16831,519),(16904,525),(16975,531),(17046,538),(17117,544),(17188,550),(17259,555),(17330,557),(17401,561),(17472,568),(17543,578),(17614,592),(17685,58),(17758,72),(17835,80),(17912,82),(17989,83),(18066,81),(18143,657),(18220,671),(18297,681),(18374,689),(18451,700),(18528,714),(18605,730),(18686,747),(18757,766),(18828,783),(18899,799),(18970,818),(19041,836),(19112,854),(19183,874),(19254,896),(19325,919),(19396,943),(19467,970),(19544,999),(19610,1027),(19676,1057),(19742,1089),(19808,1123),(19874,1162),(19940,1202),(20006,1239),(20072,1226),(20138,1217),(20204,1212),(20270,1210),(20351,1206),(20422,1202),(20493,1199),(20564,1197),(20635,1195),(20706,1193),(20777,1192),(20848,1191),(20919,1189),(20990,1188),(21061,1187),(21132,1186),(21208,1185),(21278,1184),(21348,1184),(21418,1183),(21488,1183),(21558,1182),(21628,1181),(21698,1180),(21768,1179),(21838,1178),(21908,1177),(21978,1177),(22051,1177),(22127,1178),(22203,1179),(22279,1181),(22355,1182),(22431,1180),(22507,494),(22583,483),(22659,484),(22735,493),(22811,499),(22887,499),(22968,499),(23039,497),(23110,495),(23181,487),(23252,472),(23323,459),(23394,457),(23465,456),(23536,464),(23607,475),(23678,486),(23749,492),(23826,499),(23892,520),(23958,532),(24024,531),(24090,520),(24156,511),(24222,506),(24288,506),(24354,511),(24420,518),(24486,1335),(24552,1356),(24629,1358),(24701,1326),(24773,1299),(24845,1277),(24917,1257),(24989,1237),(25061,1218),(25133,1199),(25205,1178),(25277,1145),(25349,1129),(25421,1113),(25490,1097),(25555,1082),(25620,1069),(25685,1057),(25750,1043),(25815,1030),(25880,1018),(25945,1006),(26010,995),(26075,985),(26140,974),(26205,963),(26279,953),(26350,944),(26421,934),(26492,925),(26563,917),(26634,909),(26705,901),(26776,893),(26847,886),(26918,879),(26989,872),(27060,865),(27137,859),(27208,852),(27279,847),(27350,841),(27421,836),(27492,831),(27563,826),(27634,821),(27705,819),(27776,815),(27847,812),(27918,810),(27993,808),(28069,806),(28145,803),(28221,800),(28297,794),(28373,792),(28449,788),(28525,784),(28601,781),(28677,778),(28753,774),(28829,770),(28904,766),(28975,762),(29046,758),(29117,754),(29188,752),(29259,749),(29330,747),(29401,745),(29472,743),(29543,741),(29614,739),(29685,738),(29762,736),(29828,735),(29894,733),(29960,732),(30026,731),(30092,730),(30158,730),(30224,730),(30290,730),(30356,730),(30422,730),(30488,730),(30561,730),(30632,729),(30703,729),(30774,729),(30845,730),(30916,731),(30987,732),(31058,733),(31129,734),(31200,733),(31271,732),(31342,731),(31422,730),(31493,729),(31564,729),(31635,730),(31706,732),(31777,735),(31848,737),(31919,740),(31990,743),(32061,746),(32132,749),(32203,753),(32275,756),(32346,759),(32417,762),(32488,766),(32559,770),(32630,774),(32701,778),(32772,782),(32843,786),(32914,793),(32985,798),(33056,804),(33133,809),(33204,815),(33275,821),(33346,827),(33417,834),(33488,839),(33559,846),(33630,852),(33701,859),(33772,865),(33843,872),(33914,880),(33991,887),(34067,895),(34143,903),(34219,912),(34295,922),(34371,931),(34447,942),(34523,952),(34599,954),(34675,955),(34751,945),(34827,914),(34904,876),(34975,862),(35046,853),(35117,851),(35188,848),(35259,844),(35330,826),(35401,803),(35472,781),(35543,773),(35614,766),(35685,760),(35762,755),(35834,0),(35906,77),(35978,78),(50,78),(122,77),(194,77),(266,83),(338,83),(410,77),(482,58),(554,678),(633,671),(704,665),(775,659),(846,658),(917,657),(988,657),(1059,655),(1130,652),(1201,649),(1272,646),(1343,643),(1414,640),(1494,637),(1558,634),(1622,631),(1686,628),(1750,625),(1814,622),(1878,619),(1942,617),(2006,614),(2070,612),(2134,610),(2198,608),";
    char* token = strtok(&input[0], "(),");
    while (token != nullptr) {
        uint16_t angle = atof(token) / 100.0; 
        token = strtok(nullptr, "(),");
        uint16_t distance = atof(token) / 10.0;
        token = strtok(nullptr, "(),");

        LidarPoint lidarPoint(distance, 1, angle);
        if(lidarPoint.distance() > 10) { // on ne prend pas les points < 10cm
          points.push_back(lidarPoint); 
          if(lidarPoint.distance() > distance_max) {
            distance_max = lidarPoint.distance();
          }        
        }
    }
  }

  // coordonnées cartésiennes :
  std::vector<Vector2> points_cart;
  double sumX = 0.0, sumY = 0.0;
  for (size_t i = 0; i < points.size(); i++) {
    LidarPoint lidarPoint = points[i];
    float x = lidarPoint.distance() * std::cos(lidarPoint.angle() / 180.0 * M_PI);
    float y = lidarPoint.distance() * std::sin(lidarPoint.angle() / 180.0 * M_PI);
    points_cart.push_back(Vector2(x, y));
    sumX += x;
    sumY += y;
  }

  // SerialDebug.println("*** points en coordonnées cartésiennes ***");
  // String log2 = "";
  // for (size_t i = 0; i < points_cart.size(); i++) {
  //   Vector2 point = points_cart[i];
  //   log2 += "(" + String(point.x()) + "," + String(point.y()) + ")," ;
  // }
  // SerialDebug.println(log2);

  // SerialDebug.println("distance_max: " + String(distance_max));

  int numRho = distance_max;
  int numTheta = 180;
  double thetaStep = 1 * M_PI / 180.0;
  int threshold = 30;

  // int numRho = 4;
  // points_cart.clear();
  // points_cart.push_back(Vector2(0, -4));
  // points_cart.push_back(Vector2(1, -3));
  // points_cart.push_back(Vector2(2, -2));
  // points_cart.push_back(Vector2(3, -1));
  // points_cart.push_back(Vector2(4, 0));

  std::vector<Line> lines = houghTransform(points_cart, numRho, numTheta, thetaStep, threshold);
  
  // Tri des lignes selon nb_accumulators du plus grand au plus petit
  std::sort(lines.begin(), lines.end(), [](const Line& a, const Line& b) {
      return a.nb_accumulators > b.nb_accumulators;
  });

  std::vector<Line> walls;
  double rhoTolerance = 30.0; 
  double thetaTolerance = 0.6;
  double thetaTolerancePerpendiculaire = 0.2;

  for (const auto& line : lines) {
    bool isDifferentEnough = true;
    bool isParallelOrPerpendicular = false;

    for (const auto& wall : walls) {
        double angleDifference = std::abs(line.theta - wall.theta);
        
        if (std::abs(line.rho - wall.rho) < rhoTolerance && angleDifference < thetaTolerance) {
            isDifferentEnough = false;
        }

        if (std::abs(angleDifference - M_PI / 2) < thetaTolerancePerpendiculaire) {
            isParallelOrPerpendicular = true; // perpendiculaire
        }

        if (angleDifference < thetaTolerance) {
            isParallelOrPerpendicular = true; // parallèle
        }
    }

    if (walls.size() == 0 || (isDifferentEnough && isParallelOrPerpendicular)) {
        // SerialDebug.println("ligne ajoutée -> rho: " + String(line.rho) + ", theta: " + String(line.theta) + ", accumulators: " 
          // + String(line.nb_accumulators));
        walls.push_back(line);
    }

    if(walls.size() == 4) {
      break;
    }
  }

  // Afficher les lignes détectées
  // SerialDebug.println("lines: " + String(lines.size()));
  // std::vector<double> rhos;
  // std::vector<double> thetas;
  // for (const auto& line : lines) {
      // SerialDebug.println("line -> rho: " + String(line.rho) + ", theta: " + String(line.theta) + ", accumulators: " 
          // + String(line.nb_accumulators));
      // rhos.push_back(line.rho);
      // thetas.push_back(line.theta);
  // }

  // Afficher les murs détectés
  // SerialDebug.println("*** walls: " + String(walls.size()));
  // for (const auto& wall : walls) {
  //     SerialDebug.println("wall -> rho: " + String(wall.rho) + ", theta: " + String(wall.theta) + ", accumulators: " 
  //         + String(wall.nb_accumulators));
  //     rhos.push_back(wall.rho);
  //     thetas.push_back(wall.theta);
  // }

  // String s_rhos = "";
  // for (size_t i = 0; i < rhos.size(); i++) {
  //   s_rhos += String(rhos[i] * 10) + ",";
  // }
  // SerialDebug.println("** rhos: " + s_rhos);

  // String s_thetas = "";
  // for (size_t i = 0; i < thetas.size(); i++) {
  //   s_thetas += String(thetas[i]) + ",";
  // }
  // SerialDebug.println("** thetas: " + s_thetas);

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

  // String s_corners_x = "";
  // for (size_t i = 0; i < corners.size(); i++) {
  //   s_corners_x += String(corners[i].x()) + ",";
  // }
  // SerialDebug.println("** corners x: " + s_corners_x);

  // String s_corners_y = "";
  // for (size_t i = 0; i < corners.size(); i++) {
  //   s_corners_y += String(corners[i].y()) + ",";
  // }
  // SerialDebug.println("** corners y: " + s_corners_y);

  // Centre du terrain : 
  Vector3 centroid = computeCentroid(corners);
  // SerialDebug.println("Centre du terrain : (" + String(centroid.x) + ", " + String(centroid.y) + ")");

  // orientation (angle avec le côté du terrain le plus long)
  double maxDistance = 0;
  int firstCornerIndex;
  int secondCornerIndex;

  sortPointsClockwise(corners);
  for (size_t i = 0; i < corners.size(); i++) {
    // SerialDebug.println("corner " + String(i) + ": x=" + String(corners[i].x) + ", y=" + String(corners[i].y));
    size_t nextIndex = (i + 1) % corners.size();
    double currentDistance = distance3(corners[i], corners[nextIndex]);
    // SerialDebug.println("currentDistance: " + String(currentDistance));

    double deltaY = corners[nextIndex].y - corners[i].y;
    double deltaX = corners[nextIndex].x - corners[i].x;
    double angleRadians = std::atan2(deltaY, deltaX);
    // SerialDebug.println("orientation: " + String(angleRadians * (180.0 / M_PI)) + "°"); 

    if (currentDistance > maxDistance) {
      maxDistance = currentDistance;
      firstCornerIndex = i;
      secondCornerIndex = (i + 1) % corners.size();      
    }
  }  

  // SerialDebug.println("max distance: " + String(maxDistance));
  double deltaY = corners[secondCornerIndex].y - corners[firstCornerIndex].y;
  double deltaX = corners[secondCornerIndex].x - corners[firstCornerIndex].x;
  double angleRadians = std::atan2(deltaY, deltaX); // renvoie un angle entre -PI et +PI
  if (angleRadians < 0) {
      angleRadians += M_PI; // Convertir l'angle négatif en angle positif
    }
  // SerialDebug.println("orientation: " + String(angleRadians * (180.0 / M_PI)) + "°");

  FieldInfos infos;
  infos.center = centroid;
  infos.orientation = angleRadians;
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

    x = (l1.rho * 10 * sinTheta2 - l2.rho * 10 * sinTheta1) / determinant;
    y = (l2.rho * 10 * cosTheta1 - l1.rho * 10 * cosTheta2) / determinant;

    return true; 
}*/