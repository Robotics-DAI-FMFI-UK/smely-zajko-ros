#ifndef _PLANNER_H_
#define _PLANNER_H_

#include <vector>


#include "LocalMap.h"

struct Bod {
    int first;
    int second;
};

class Planner {

public:

    Planner(LocalMap *localMap);

    void findBestHeading_graph(int random);

private:

    LocalMap *localMap;

    int dlzka_useku = 60; //20; //1;  // 5;
    int saferange = 10; //1; //3;
    int nearest = 200; //100; //16;
    int multiplier = 5;
    int stredove_body_max = 50; // maximalny pocet novych stredovych bodov

    int DISTANCE_TO_SMOOTH_CROSSING_CM = 500;

    double zjazdny_teren=0.9;   // maximum number of problems found on a corridor from robot to some chosen point on planned trajectory that will be taken as the first to go to
    double maximalna_vzdialenost=300.0;  // how far in dm can the first chosen point on trajectory be from current robot position

    //stredove_body_old v sebe obsahuju body zo slimak_trajektory a pametam si poslednych X bodov kde X je hodnota v premennej max_pocet_stredove_body_old
    vector<pair<int, int>> stredove_body_old; // vektor obsahuje vysledne body trajektorie konkretne X poslednych kde X je v hodnote max_pocet_stredove_body_old
    int pocet_stredove_body_old = 0;//pomocna premenna
    int max_pocet_stredove_body_old = 10;
    int old = 1; // prepinac ci si chceme pamatat stare trajektorie s dlzkou pamate max_pocet_stredove_body_old poslednych hodnot

    int res_short = 1;  // shortening the planned trajectory to avoid too steep turning and the resulting slalom

    int cena_advanced = 1; // prepinac ci chceme pouzivat vylepsenu cenu cesty
    int lidar_penalizacia = 10; // kazdy pixel na ceste ktory obsahuje prekazku z liradu je 10x drahsi na prejazd
    double camera_penalizacia = 2.5; // kazdy pixel na ceste ktory obsahuje travu je pocitany (1-hodnota z matrix_cam) * X kde X je hodnota ktorou nasobime penalizaciu za prechod cez travu

    int bezier_switch = 1; // zapina a vypina kreslenie bezierovej kryvky na smoothing
    int bezier_number_of_points = 20; // pocet vrcholov od zaciatku pre ktore vytvaram krivku

    int vocal = 0; //parameter nastavujuci intenzitu vypisov 0 alebo 1



    static const int pocet_priamok = 150;//100


    void generuj_nahodne(int dvojice_nahodnych_bodov_na_okraji_mapy[pocet_priamok][2][2], int pocet_dvojic);

    void generuj_pravidelne(int dvojice_nahodnych_bodov_na_okraji_mapy[pocet_priamok][2][2], int pocet_dvojic);

    void generuj_kostru_grafu(Graph &graph, vector<pair<int, int>> *stredove_body, int pocet_bodov_grafu,
                              double **cena_cesty);

    double vzdialenost_bodov(int *A, int *B);

    void kontroluj_zjazdnost(int dvojice_nahodnych_bodov_na_okraji_mapy[pocet_priamok][2][2], int n,
                             vector<pair<Bod, Bod>> *dvojice);

    void najdi_stredove_body_a_ceny(vector<pair<int, int>> *stredove_body, double **cena_cesty,
                                    vector<pair<Bod, Bod>> *pretnute_okraje_zjazdnej_casti);

    void napln_graf(Graph &graph, vector<pair<int, int>> *stredove_body, int size_stredove_body, double **cena_cesty);

    void bezier(vector<pair<int, int>> *trajektoria, vector<pair<int, int>> *bezier_body);

    void sprav_diagnostiku(bool diagnostika, const char *param,
                           int dvojice_nahodnych_bodov_na_okraji_mapy[pocet_priamok][2][2],
                           int pocet_dvojice_nahodnych_bodov_na_okraji_mapy,
                           vector<pair<Bod, Bod>> *pretnute_okraje_zjazdnej_casti,
                           vector<pair<int, int>> *stredove_body, vector<pair<int, int>> *bezier_body);

    void find_border_point_for_angle(double wished_heading, int goal_position[]);

    double trasa_je_cista(int start[2], int end[2], int x);
    double trasa_je_cista_box(int start[2], int end[2], int half_robot_width);

    void skratenie_cesty(pair<int, vector<int>> *result, pair<int, vector<int>> *result_short, vector<pair<int, int>> *stredove_body);

};

#endif
