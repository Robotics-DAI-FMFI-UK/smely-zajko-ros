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

    int dlzka_useku = 20;//1  // 5
    int saferange = 10;//1  // 3
    int nearest = 60;//100 //16
    int multiplier = 5;

    //stredove_body_old v sebe obsahuju body zo slimak_trajektory a pametam si poslednych X bodov kde X je hodnota v premennej max_pocet_stredove_body_old
    vector<pair<int, int>> stredove_body_old; // vektor obsahuje vysledne body trajektorie konkretne X poslednych kde X je v hodnote max_pocet_stredove_body_old
    int pocet_stredove_body_old = 0;//pomocna premenna
    int max_pocet_stredove_body_old = 20;
    int old = 1; // prepinac ci si chceme pamatat stare trajektorie s dlzkou pamate max_pocet_stredove_body_old poslednych hodnot

    int cena_advanced = 0; // prepinac ci chceme pouzivat vylepsenu cenu cesty
    int lidar_penalizacia = 10; // kazdy pixel na ceste ktory obsahuje prekazku z liradu je 10x drahsi na prejazd
    int camera_penalizacia = 4; // kazdy pixel na ceste ktory obsahuje travu je pocitany (1-hodnota z matrix_cam) * X kde X je hodnota ktorou nasobime penalizaciu za prechod cez travu

    int bezier_switch = 0; // zapina a vypina kreslenie bezierovej kryvky na smoothing




    static const int pocet_priamok = 150;//100


    void generuj_nahodne(int dvojice_nahodnych_bodov_na_okraji_mapy[pocet_priamok][2][2], int pocet_dvojic);
    void generuj_pravidelne(int dvojice_nahodnych_bodov_na_okraji_mapy[pocet_priamok][2][2], int pocet_dvojic);
    void generuj_kostru_grafu(Graph &graph, vector<pair<int, int>> *stredove_body, int pocet_bodov_grafu, double **cena_cesty);
    double vzdialenost_bodov(int *A, int *B);
    void kontroluj_zjazdnost(int dvojice_nahodnych_bodov_na_okraji_mapy[pocet_priamok][2][2], int n, vector<pair<Bod, Bod>> *dvojice);
    void najdi_stredove_body_a_ceny(vector<pair<int, int>> *stredove_body, double **cena_cesty,
                                    vector<pair<Bod, Bod>> *pretnute_okraje_zjazdnej_casti);
    void napln_graf(Graph &graph, vector<pair<int, int>> *stredove_body, int size_stredove_body, double **cena_cesty);
    void bezier(vector<pair<int, int>> *trajektoria, vector<pair<int, int>> *bezier_body);
    void sprav_diagnostiku(bool diagnostika, const char *param, int dvojice_nahodnych_bodov_na_okraji_mapy[pocet_priamok][2][2],
                           int pocet_dvojice_nahodnych_bodov_na_okraji_mapy,
                           vector<pair<Bod, Bod>> *pretnute_okraje_zjazdnej_casti, vector<pair<int, int>> *stredove_body, vector<pair<int, int>> *bezier_body);
    void find_border_point_for_angle(double wished_heading, int goal_position[]);

};
#endif
