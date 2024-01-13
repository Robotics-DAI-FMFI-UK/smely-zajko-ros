#include <iostream>
#include <vector>
#include <algorithm>
#include <queue>
#include <math.h>

#include "Planner.h"
#include "LocalMap.h"
#include "Graph.h"

using namespace cv;

Planner::Planner(LocalMap *localMap_reference)
{
  localMap = localMap_reference;
}

void Planner::generuj_nahodne(int dvojice_nahodnych_bodov_na_okraji_mapy[pocet_priamok][2][2], int pocet_dvojic) {
    for (int i = 0; i < pocet_dvojic; i++) {
        //prvy bod A1 leziaci na hrane mapy
        int strana = rand() % 4;
        int druha_strana = (strana + ((rand() % 3) + 1)) % 4;
        if (strana == 0) {
            dvojice_nahodnych_bodov_na_okraji_mapy[i][0][0] = rand() % gridWidth;
            dvojice_nahodnych_bodov_na_okraji_mapy[i][0][1] = 0;
        } else if (strana == 2) {
            dvojice_nahodnych_bodov_na_okraji_mapy[i][0][0] = rand() % gridWidth;
            dvojice_nahodnych_bodov_na_okraji_mapy[i][0][1] = gridWidth - 1;
        } else if (strana == 1) {
            dvojice_nahodnych_bodov_na_okraji_mapy[i][0][0] = gridWidth - 1;
            dvojice_nahodnych_bodov_na_okraji_mapy[i][0][1] = rand() % gridWidth;
        } else if (strana == 3) {
            dvojice_nahodnych_bodov_na_okraji_mapy[i][0][0] = 0;
            dvojice_nahodnych_bodov_na_okraji_mapy[i][0][1] = rand() % gridWidth;
        }

        if (druha_strana == 0) {
            dvojice_nahodnych_bodov_na_okraji_mapy[i][1][0] = (rand() % gridWidth);
            dvojice_nahodnych_bodov_na_okraji_mapy[i][1][1] = 0;
        } else if (druha_strana == 2) {
            dvojice_nahodnych_bodov_na_okraji_mapy[i][1][0] = (rand() % gridWidth);
            dvojice_nahodnych_bodov_na_okraji_mapy[i][1][1] = (gridWidth - 1);
        } else if (druha_strana == 1) {
            dvojice_nahodnych_bodov_na_okraji_mapy[i][1][0] = (gridWidth - 1);
            dvojice_nahodnych_bodov_na_okraji_mapy[i][1][1] = (rand() % gridWidth);
        } else if (druha_strana == 3) {
            dvojice_nahodnych_bodov_na_okraji_mapy[i][1][0] = 0;
            dvojice_nahodnych_bodov_na_okraji_mapy[i][1][1] = (rand() % gridWidth);
        }
    }
}

void Planner::generuj_pravidelne(int dvojice_nahodnych_bodov_na_okraji_mapy[pocet_priamok][2][2], int pocet_dvojic) {
    int pocet_na_stranu = pocet_dvojic / 4;
    int krok = (gridWidth - 40) / pocet_na_stranu;
    //generujem priamky v 45 stupnovom uhle do oboch smerov
    for (int i = 0; i < pocet_na_stranu; i++) {
        dvojice_nahodnych_bodov_na_okraji_mapy[i][0][0] = 20 + i * krok;
        dvojice_nahodnych_bodov_na_okraji_mapy[i][0][1] = 0;
        dvojice_nahodnych_bodov_na_okraji_mapy[i][1][0] = 0;
        dvojice_nahodnych_bodov_na_okraji_mapy[i][1][1] = 20 + i * krok;
    }
    for (int i = 0; i < pocet_na_stranu; i++) {
        dvojice_nahodnych_bodov_na_okraji_mapy[i][0][0] = 20 + i * krok;
        dvojice_nahodnych_bodov_na_okraji_mapy[i][0][1] = gridWidth;
        dvojice_nahodnych_bodov_na_okraji_mapy[i][1][0] = gridWidth;
        dvojice_nahodnych_bodov_na_okraji_mapy[i][1][1] = 20 + i * krok;
    }
    for (int i = 0; i < pocet_na_stranu; i++) {
        dvojice_nahodnych_bodov_na_okraji_mapy[i][0][0] = 0;
        dvojice_nahodnych_bodov_na_okraji_mapy[i][0][1] = 20 + i * krok;
        dvojice_nahodnych_bodov_na_okraji_mapy[i][1][0] = 20 + i * krok;
        dvojice_nahodnych_bodov_na_okraji_mapy[i][1][1] = 0;
    }
    for (int i = 0; i < pocet_na_stranu; i++) {
        dvojice_nahodnych_bodov_na_okraji_mapy[i][0][0] = gridWidth;
        dvojice_nahodnych_bodov_na_okraji_mapy[i][0][1] = 20 + i * krok;
        dvojice_nahodnych_bodov_na_okraji_mapy[i][1][0] = 20 + i * krok;
        dvojice_nahodnych_bodov_na_okraji_mapy[i][1][1] = gridWidth;
    }
    for (int i = 4 * pocet_na_stranu; i < pocet_dvojic; i++) {// doplnenie priamok nech ich mam celkovo n
        dvojice_nahodnych_bodov_na_okraji_mapy[i][0][0] = 0;
        dvojice_nahodnych_bodov_na_okraji_mapy[i][0][1] = gridWidth;
    }
}

void
Planner::generuj_kostru_grafu(Graph graph, vector<pair<int, int>> stredove_body, int pocet_bodov_grafu, double **cena_cesty) {
    int navstivene[pocet_bodov_grafu];
    int pocet_kostra = 0;
    vector<int> kostra;
    for (int i = 0; i < pocet_bodov_grafu; ++i) {
        navstivene[i] = 0;
    }
    navstivene[0] = 1;
    kostra.push_back(0);
    //kostra.push_back(make_pair(stredove_body[0][0], stredove_body[0][1]));
    pocet_kostra++;
    int minimum = 1000000;
    int index_prveho;
    int index_druheho;
    while (pocet_kostra < pocet_bodov_grafu) {
        minimum = 1000000;
        index_prveho = 0;
        index_druheho = 0;
        for (int i = 0; i < kostra.size(); ++i) {
            for (int j = 0; j < pocet_bodov_grafu; ++j) {
                if (minimum > cena_cesty[kostra.at(i)][j] && navstivene[j] == 0) {
                    index_prveho = i;
                    index_druheho = j;
                    minimum = cena_cesty[kostra.at(i)][j];
                }
            }
        }
        graph.addEdge(index_prveho, index_druheho, cena_cesty[index_prveho][index_druheho]);
        kostra.push_back(index_druheho);
        navstivene[index_druheho] = 1;
        pocet_kostra++;
    }
}


double Planner::vzdialenost_bodov(int *A, int *B) {
    return sqrt((A[0] - B[0]) * (A[0] - B[0]) + (A[1] - B[1]) * (A[1] - B[1]));
}

void Planner::kontroluj_zjazdnost(int dvojice_nahodnych_bodov_na_okraji_mapy[pocet_priamok][2][2], int n, vector<pair<Bod, Bod>> dvojice) {
    bool zjazdnost;
    int x1 = 0;
    int y1 = 0;
    int prvy_bod[2];
    bool prvy = false;
    int druhy_bod[2];
    bool druhy = false;
    printf("kz1\n");
    for (int i = 0; i < n; i++) {
        printf("kz(%d)\n", i);
        zjazdnost = false;
        prvy = false;
        druhy = false;
        for (int j = 0; j < gridWidth; j++) {
            x1 = int(dvojice_nahodnych_bodov_na_okraji_mapy[i][0][0] +
                     (dvojice_nahodnych_bodov_na_okraji_mapy[i][1][0] - 
                     dvojice_nahodnych_bodov_na_okraji_mapy[i][0][0]) * (j / (double)gridWidth));
            y1 = int(dvojice_nahodnych_bodov_na_okraji_mapy[i][0][1] +
                     (dvojice_nahodnych_bodov_na_okraji_mapy[i][1][1] - 
                     dvojice_nahodnych_bodov_na_okraji_mapy[i][0][1]) * (j / (double)gridWidth));
            //printf("x1=%d, y1=%d\n", x1, y1);
            if (x1 >= 0 && x1 < gridWidth && y1 >= 0 && y1 < gridWidth) {
                //printf("ojojooo\n");
                if ((localMap->matrix[x1][y1] == 0 && localMap->matrix_cam[x1][y1] > 0 && zjazdnost == false)) {
                    zjazdnost = true;
                    if (prvy == false) {
                        prvy_bod[0] = x1;
                        prvy_bod[1] = y1;
                        prvy = true;
                    }
                }
                if (((localMap->matrix[x1][y1] != 0 || localMap->matrix_cam[x1][y1] == 0) && zjazdnost == true)) {
                    zjazdnost = false;
                    if (druhy == false) {
                        druhy_bod[0] = x1;
                        druhy_bod[1] = y1;
                        druhy = true;
                    }
                    prvy = false;
                    druhy = false;
                    if (vzdialenost_bodov(prvy_bod,druhy_bod)>dlzka_useku) {
                        Bod prvy_b;
                        prvy_b.first = prvy_bod[0];
                        prvy_b.second = prvy_bod[1];
                        Bod druhy_b;
                        druhy_b.first = druhy_bod[0];
                        druhy_b.second = druhy_bod[1];

                        dvojice.emplace_back(make_pair(prvy_b, druhy_b));
                    }
                    //dvojice --------- je to vector obsahujuci vsetky dvojice bodov ktorych priamky pretli hrany zjazdnosti na 2 miestach a su dlhsie ako dlzka_useku
                }
            }
        }
    }
    printf("dvojic=%lu\n", dvojice.size());
}

void Planner::najdi_stredove_body_a_ceny(vector<pair<int, int>> stredove_body, double **cena_cesty,
                                vector<pair<Bod, Bod>> pretnute_okraje_zjazdnej_casti) {

    double start[2] = {gridWidth / 2.0, gridWidth / 2.0};
    double ciel[2] = {800, 150};
    double wished_heading = localMap->angle - localMap->compassHeading + localMap->currWayHeading;
    find_border_point_for_angle(wished_heading, ciel, gridWidth);

    int bod_a[2];
    int bod_b[2];
    int stred[2];
    stredove_body.push_back(make_pair(start[0], start[1]));
    for (int i = 0; i < pretnute_okraje_zjazdnej_casti.size(); i++) {
        bod_a[0] = pretnute_okraje_zjazdnej_casti[i].first.first;
        bod_a[1] = pretnute_okraje_zjazdnej_casti[i].first.second;
        bod_b[0] = pretnute_okraje_zjazdnej_casti[i].second.first;
        bod_b[1] = pretnute_okraje_zjazdnej_casti[i].second.second;
        stred[0] = int((bod_a[0] + bod_b[0]) / 2);
        stred[1] = int((bod_a[1] + bod_b[1]) / 2);
        stredove_body.push_back(make_pair(stred[0], stred[1]));
    }
    stredove_body.push_back(make_pair(ciel[0], ciel[1]));

    int size_stredove_body = stredove_body.size();

    for (int i = 0; i < size_stredove_body; i++)
        for (int j = 0; j < size_stredove_body; j++) {
            double cena = sqrt((stredove_body[i].first - stredove_body[j].first) * (stredove_body[i].first - stredove_body[j].first) +
                               (stredove_body[i].second - stredove_body[j].second) * (stredove_body[i].second - stredove_body[j].second));
            cena_cesty[i][j] = cena;
        }
}

void Planner::napln_graf(Graph graph, vector<pair<int, int>> stredove_body, int size_stredove_body, double **cena_cesty) {
    int startNode = 0;
    int endNode = size_stredove_body - 1;
    for (int i = 0; i < size_stredove_body; i++) {
        graph.setXY(i, stredove_body[i].first, stredove_body[i].second);
    }
    double najblizsie_k_cielu[2];
    double start[2] = {gridWidth / 2.0, gridWidth / 2.0};
    najblizsie_k_cielu[0] = start[0];
    najblizsie_k_cielu[1] = start[1];
    double najblizsi = cena_cesty[0][size_stredove_body - 1];
    int najblizsi_index = 0;


    generuj_kostru_grafu(graph, stredove_body, size_stredove_body, cena_cesty);

    //zisti_parametre_saferage();
    for (int i = 0; i < size_stredove_body; i++) {
        for (int j = i + 1; j < size_stredove_body; j++) {
            if (cena_cesty[i][j] < nearest)
                graph.addEdge(i, j, cena_cesty[i][j]);
        }
        if (najblizsi > cena_cesty[i][size_stredove_body - 1]) {
            najblizsi = cena_cesty[i][0];
            najblizsi_index = i;
            endNode = najblizsi_index;
        }
    }
}

void Planner::sprav_diagnostiku(bool diagnostika, const char *param, int dvojice_nahodnych_bodov_na_okraji_mapy[pocet_priamok][2][2],
                       int pocet_dvojice_nahodnych_bodov_na_okraji_mapy,
                       vector<pair<Bod, Bod>> pretnute_okraje_zjazdnej_casti, vector<pair<int, int>> stredove_body) {
    //bool diagnostika=true;
    //char param[9]="11111111";

    //0 body na hranach lokalnej mapy
    //1 priamky prechadzajuce cez vytvorene body
    //2 priesecniky priamok a hranice zjazdneho a nezjazdneho
    //3 usecka medzi 2 preseknutyimi bodmi
    //4 stred useciek
    //5 kruznica dosahu pre spajanie grafu
    //6 vysledny graf

// TO DO LIST
//treba generovat priamky aj pravidelneba iterovat po x aj y podla uhlu pria ***** asi hotovo *****
//optimalizacia hladania parametrov
//upravim vektor aby iteroval podla x=x0+kx*t a y=y0+ky*t ***** asi hotovo *****
//treba pridat kostru grafu primov algor...


    if (!diagnostika) return;
    double start[2] = {gridWidth / 2.0, gridWidth / 2.0};
    double ciel[2] = {800, 150};
    double wished_heading = localMap->angle - localMap->compassHeading + localMap->currWayHeading;
    find_border_point_for_angle(wished_heading, ciel, gridWidth);

    Mat image(gridWidth * multiplier, gridWidth * multiplier, CV_8UC3);
    for (int i = 0; i < image.rows; i++) {
        for (int j = 0; j < image.cols; j++) {
            if (localMap->matrix[i / 5][j / 5] == 0 && localMap->matrix_cam[i / 5][j / 5] > 0)
                image.at<Vec3b>(i, j) = Vec3b(255, 255, 255);
            else if (localMap->matrix[i / 5][j / 5] > 0)
                image.at<Vec3b>(i, j) = Vec3b(10, 250, 10);
            else if (localMap->matrix_cam[i / 5][j / 5] == 0)
                image.at<Vec3b>(i, j) = Vec3b(10, 10, 250);
            else
                image.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
        }
    }

    //pociatocne body na okrajoch
    for (int i = 0; i < pocet_dvojice_nahodnych_bodov_na_okraji_mapy; i++) {
        int bod1[2];
        int bod2[2];
        bod1[0] = dvojice_nahodnych_bodov_na_okraji_mapy[i][0][0];
        bod1[1] = dvojice_nahodnych_bodov_na_okraji_mapy[i][0][1];
        bod2[0] = dvojice_nahodnych_bodov_na_okraji_mapy[i][1][0];
        bod2[1] = dvojice_nahodnych_bodov_na_okraji_mapy[i][1][1];
        if (param[0] == '1') {
            circle(image, Point(bod1[0] * multiplier, bod1[1] * multiplier), 3, Scalar(250, 10, 10), -1);
            circle(image, Point(bod2[0] * multiplier, bod2[1] * multiplier), 3, Scalar(250, 10, 10), -1);
        }
        if (param[1] == '1') {
            line(image, Point(bod1[0] * multiplier, bod1[1] * multiplier), Point(bod2[0] * multiplier, bod2[1] * multiplier), Scalar(255, 0, 0), 1);
        }
    }
    for (int i = 0; i < pretnute_okraje_zjazdnej_casti.size(); i++) {

        if (param[2] == '1') {
            circle(image, Point(multiplier * pretnute_okraje_zjazdnej_casti[i].first.first,
                                multiplier * pretnute_okraje_zjazdnej_casti[i].first.second), 3, Scalar(100, 10, 10), -1);
            circle(image, Point(multiplier * pretnute_okraje_zjazdnej_casti[i].second.first,
                                multiplier * pretnute_okraje_zjazdnej_casti[i].second.second), 3, Scalar(100, 10, 10), -1);
        }
        if (param[3] == '1') {
            line(image,
                 Point(multiplier * pretnute_okraje_zjazdnej_casti[i].first.first, multiplier * pretnute_okraje_zjazdnej_casti[i].first.second),
                 Point(multiplier * pretnute_okraje_zjazdnej_casti[i].second.first,
                       multiplier * pretnute_okraje_zjazdnej_casti[i].second.second), Scalar(100, 100, 0), 1);
        }
    }

    for (int i = 0; i < stredove_body.size(); i++) {

        if (param[4] == '1') {
            circle(image, Point(multiplier * stredove_body[i].first, multiplier * stredove_body[i].second), 3, Scalar(250, 10, 250), -1);
        }
        if (param[5] == '1') {
            circle(image, Point(300, 300), nearest * multiplier, Scalar(255, 0, 0), 1);
        }
    }
    int bodA[2];
    bodA[0] = start[0];
    bodA[1] = start[1];
    int bodB[2];
    bodB[0] = start[0];
    bodB[1] = start[1];

    for (int i = 0; i < localMap->slimak_trajectory.size(); i++) {
        if (param[6] == '1') {
            bodA[0] = localMap->slimak_trajectory[i].first;
            bodA[1] = localMap->slimak_trajectory[i].second;
            line(image, Point(multiplier * bodB[0], multiplier * bodB[1]), Point(multiplier * bodA[0], multiplier * bodA[1]), Scalar(255, 0, 0), 10);
            bodB[0] = bodA[0];
            bodB[1] = bodA[1];
        }
    }


    imshow("Obrazok", image);
    waitKey(1);
}

void LocalMap::find_border_point_for_angle(double wished_heading, double goal_position[], double map_width)
{
        double target_x, target_y;
        double epsilon_tan = 0.0001;

    // ciel je pred nami pod uhlom +- 90 stupov
        if (fabs(wished_heading - M_PI_2) < epsilon_tan)
        {
                if (wished_heading > M_PI)   // +90
                {
                  target_x = map_width;
                  target_y = int(map_width/2);
            }
            else   // -90
            {
                  target_x = 0;
                  target_y = int(map_width/2);
            }
        }
        else if (angleDiffAbs(wished_heading, 0) <= M_PI_4)  // horny kvandrant (+- 45 stupnov)
        {
                target_x = map_width / 2 + map_width * tan(wished_heading) / 2;
                target_y = map_width;
        }
        else if (fabs(wished_heading - M_PI_2) <= M_PI_4)   // pravy kvadrant (45..135)
        {
                target_x = map_width;
                target_y = map_width / 2 + map_width * tan(wished_heading - M_PI_2) / 2;
        }
        else if (fabs(wished_heading - M_PI) <= M_PI_4)   // dolny kvadrant (135..225)
        {
                target_x = map_width / 2 + map_width * tan(wished_heading - M_PI) / 2;
                target_y = map_width;
        }
        else if (fabs(wished_heading - M_PI_2 - M_PI_4) <= M_PI_4)   // lavy kvadrant (225..315)
        {
                target_x = 0;
                target_y = map_width / 2 + map_width * tan(wished_heading - M_PI_2 - M_PI) / 2;
        }

        goal_position[0] = target_x;
        goal_position[1] = target_y;

}

void Planner::find_border_point_for_angle(double wished_heading, double goal_position[], double map_width)
{
        double target_x, target_y;
        double epsilon_tan = 0.0001;

    // ciel je pred nami pod uhlom +- 90 stupov
        if (fabs(wished_heading - M_PI_2) < epsilon_tan)
        {
                if (wished_heading > M_PI)   // +90
                {
                  target_x = map_width;
                  target_y = int(map_width/2);
            }
            else   // -90
            {
                  target_x = 0;
                  target_y = int(map_width/2);
            }
        }
        else if (angleDiffAbs(wished_heading, 0) <= M_PI_4)  // horny kvandrant (+- 45 stupnov)
        {
                target_x = map_width / 2 + map_width * tan(wished_heading) / 2;
                target_y = map_width;
        }
        else if (fabs(wished_heading - M_PI_2) <= M_PI_4)   // pravy kvadrant (45..135)
        {
                target_x = map_width;
                target_y = map_width / 2 + map_width * tan(wished_heading - M_PI_2) / 2;
        }
        else if (fabs(wished_heading - M_PI) <= M_PI_4)   // dolny kvadrant (135..225)
        {
                target_x = map_width / 2 + map_width * tan(wished_heading - M_PI) / 2;
                target_y = map_width;
        }
        else if (fabs(wished_heading - M_PI_2 - M_PI_4) <= M_PI_4)   // lavy kvadrant (225..315)
        {
                target_x = 0;
                target_y = map_width / 2 + map_width * tan(wished_heading - M_PI_2 - M_PI) / 2;
        }

        goal_position[0] = target_x;
        goal_position[1] = target_y;

}


void Planner::findBestHeading_graph() {


    double start[2] = {gridWidth / 2.0, gridWidth / 2.0};
    double ciel[2] = {800, 150};
    double cielovapozicia_x, cielovapozicia_y;
    double wished_heading = localMap->angle - localMap->compassHeading + localMap->currWayHeading;

    printf("chkpt1\n");
    find_border_point_for_angle(wished_heading, ciel, gridWidth); 
    //funkcia najde bod na hrane lokalnej mapy najblizsie k aktualnemu cielu

    int dvojice_nahodnych_bodov_na_okraji_mapy[pocet_priamok][2][2];
    int pocet_dvojice_nahodnych_bodov_na_okraji_mapy = pocet_priamok;
    //pole obsahuje 2 body so suradnicami x a y

    bool nahodne=true;
    if (nahodne)
    {
        printf("chkpt2\n");
        generuj_nahodne(dvojice_nahodnych_bodov_na_okraji_mapy, pocet_dvojice_nahodnych_bodov_na_okraji_mapy);
        //funkcia generuje do pola dvojice_nahodnych_bodov_na_okraji_mapy dvojice nahodnych bodov na hranach lokalnej mapy
    }
    else
    {
        printf("chkpt2b\n");
        generuj_pravidelne(dvojice_nahodnych_bodov_na_okraji_mapy,pocet_dvojice_nahodnych_bodov_na_okraji_mapy);
        //funkcia generuje do pola dvojice_nahodnych_bodov_na_okraji_mapy dvojice pravidelnych bodov na hranach lokalnej mapy pod 45 stupnovim uhlom
    }

    vector<pair<Bod, Bod>> pretnute_okraje_zjazdnej_casti;
    //pole pretnute_okraje_zjazdnej_casti obsahuje dvojice bodov ktore reprezentuju priesecniky priamky vytvorenej pomocou bodov predchadzajucej
    //  funkcie generuj_nahodne/generuj_pravidelne

    printf("chkpt3\n");
    kontroluj_zjazdnost(dvojice_nahodnych_bodov_na_okraji_mapy, pocet_dvojice_nahodnych_bodov_na_okraji_mapy,
                        pretnute_okraje_zjazdnej_casti);
    //Funkcia kontroluj_zjazdnost vytvori priamky zo vztupneho pola bodov. vytvori rovnicu priamky pre kazdu dvojicu bodov a nasledne cez ne iteruje
    //  a kontroluje ci priamka prechadza cez zjazdnu alebo nezjazdnu cast mapy. pri kazdej zmene stavu zjazdna/nezjazdna zaznamena bod a ked najde
    //  2 body tak ich priradi do pola pretnute_okraje_zjazdnej_casti ako dvojicu bodov

    //zisti_parametre_sirka(pretnute_okraje_zjazdnej_casti);
    //Funkcia zistuje aky siroky usek by mala zvolit aby to boli relevantne data
    //Zatial nefunguje

    vector<pair<int, int>> stredove_body;
    //pole stredove_body obsahuje vsetky body ktore su v strede zjazdnej casti

    printf("chkpt4\n");
    int cena_cesty_size = pretnute_okraje_zjazdnej_casti.size() + 2;
    double *cena_cesty[cena_cesty_size];
    printf("chkpt5\n");
    for (int i = 0; i < cena_cesty_size; i++)
      cena_cesty[i] = new double[pretnute_okraje_zjazdnej_casti.size() + 2];
    printf("chkpt6\n");

    //pole cena_cesty obsahuje 2 rozmerne pole a je to matica cier z bodu i do bodu j. pocet vsetkych stredovych bodov +1 za pociatocny startovaci bod a +1 za cielovy bod kam sa chcem dostat

    najdi_stredove_body_a_ceny(stredove_body, cena_cesty, pretnute_okraje_zjazdnej_casti);
    //funkcia dosadi do pola stredove body vsetky stredove body a nasledne spocita vsetky ceny z bodu i do bodu j aby som neskor nemusel pocitat znova tieto vzdialenosti

    printf("chkpt7\n");
    int size_stredove_body = stredove_body.size();
    int startNode = 0;
    int endNode = size_stredove_body - 1;

    if (size_stredove_body > 1)
    {
      printf("chkpt8\n");
      Graph graph(size_stredove_body);
      printf("chkpt9\n");
      napln_graf(graph, stredove_body, size_stredove_body, cena_cesty);
      //funkcia do premennej graf vlozi vsetky hrany ktore su medzi bodmi a ich dlzka je mensia ako parameter nearest
  
  
      // v tejto casti dijkstrovym algoritmom hladam v dvojrozmernom poli mapa[n][n] najkratsiu cestu k najblizsiemu bodu k cielu co moze byt okraj lokalnej mapy robota
  
      printf("dijkstra() bdefor\n");
      printf("startNode=%d, endNode=%d\n", startNode, endNode);
  
      localMap->visualizedGraph = graph;
  
      pair<int, vector<int>> result = graph.dijkstra(startNode, endNode);
      //do premennej result vkladam pocet a vektor zoradenych bodov grafu ktory reprezentuje najkratsiu cestu v grafe
  
      printf("dijkstra() aftr cost=%d, vecsize=%lu\n", result.first, result.second.size());
      // v premennej result by mala byt cesta do ciela
      localMap->slimak_trajectory.clear();
  
      pthread_mutex_lock(&localMap->trajectory_lock);
      for (vector<int>::iterator i = result.second.begin(); i < result.second.end(); i++)
          localMap->slimak_trajectory.push_back(make_pair(stredove_body[*i].first, stredove_body[*i].second));
      pthread_mutex_unlock(&localMap->trajectory_lock);
  
      double x_prvy = localMap->slimak_trajectory[0].first;
      double y_prvy = localMap->slimak_trajectory[0].second;
      double x_druhy = localMap->slimak_trajectory[1].first;
      double y_druhy = localMap->slimak_trajectory[1].second;
  
      localMap->bestSlimakHeading = atan2((y_druhy - y_prvy), (x_druhy - x_prvy));
      printf("leaving findbestS\n");
  
    }
    sprav_diagnostiku(true, "11111000", dvojice_nahodnych_bodov_na_okraji_mapy,
                        pocet_dvojice_nahodnych_bodov_na_okraji_mapy, pretnute_okraje_zjazdnej_casti, stredove_body);
      //Funkcia sprav_diagnostiku je nastroj na debugovanie v ktorom vidime graficky znazornene jednotlive kroky algoritmu.
      //  vykresluje najprv nahodne/pravidelne body na hranach mapy, priamky ktore preseknu lokalnu mapu,priesecniky zjaznej casti
      //  nasledne usecky ktore su tvorene priesecnikmi zjazdnej a nezjazdnej casti. stredove body, kruznice zo stredovych bodov
      //  znazornujuce hranice vzdialenosti ku ktorim vedia spravit hranu grafu nakoniec aj vsetky cesty ktore
      //  su zjazdne

    printf("chkpt10\n");
    for (int i = 0; i < cena_cesty_size; i++)
       delete [] cena_cesty[i];
    printf("chkpt11\n");
}
