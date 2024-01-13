#ifndef _PLANNER_H_
#define _PLANNER_H_

#include "LocalMap.h"

struct Bod {
    int first;
    int second;
};

class Planner {

public:

  Planner(LocalMap *localMap);

  void findBestHeading_graph();

private:

  LocalMap *localMap;

  int dlzka_useku = 1;  // 5
  int saferange = 1;  // 3
  int nearest = 30; //16
  int multiplier = 5;

  static const int pocet_priamok = 100;


  void generuj_nahodne(int dvojice_nahodnych_bodov_na_okraji_mapy[pocet_priamok][2][2], int pocet_dvojic);
  void generuj_pravidelne(int dvojice_nahodnych_bodov_na_okraji_mapy[pocet_priamok][2][2], int pocet_dvojic);
  void generuj_kostru_grafu(Graph graph, vector<pair<int, int>> stredove_body, int pocet_bodov_grafu, double **cena_cesty);
  double vzdialenost_bodov(int *A, int *B);
  void kontroluj_zjazdnost(int dvojice_nahodnych_bodov_na_okraji_mapy[pocet_priamok][2][2], int n, vector<pair<Bod, Bod>> dvojice);
  void najdi_stredove_body_a_ceny(vector<pair<int, int>> stredove_body, double **cena_cesty,
                                vector<pair<Bod, Bod>> pretnute_okraje_zjazdnej_casti);
  void napln_graf(Graph graph, vector<pair<int, int>> stredove_body, int size_stredove_body, double **cena_cesty);
  void sprav_diagnostiku(bool diagnostika, const char *param, int dvojice_nahodnych_bodov_na_okraji_mapy[pocet_priamok][2][2],
                       int pocet_dvojice_nahodnych_bodov_na_okraji_mapy,
                       vector<pair<Bod, Bod>> pretnute_okraje_zjazdnej_casti, vector<pair<int, int>> stredove_body);
  void find_border_point_for_angle(double wished_heading, double goal_position[], double map_width);

};
#endif
