#include "Zed.h"
#include <sl/Camera.hpp>
#include <vector>
#include <opencv2/opencv.hpp>


using namespace std;
using namespace sl;
using namespace cv;

double prekazka = 0.05;
double prekazka_postupna = 0.08;

static const int gridSize = GRID_SIZE;
static const int gridWidth = GRID_WIDTH;
static const int gridHeight = GRID_HEIGHT;

static const int multiplier = 5;

int main(int argc, char **argv) {

    // Create a ZED camera object
    Camera zed;

    // Set configuration parameters
    InitParameters init_parameters;
    init_parameters.depth_mode = DEPTH_MODE::ULTRA; // Use ULTRA depth mode
    init_parameters.coordinate_units = UNIT::MILLIMETER; // Use millimeter units (for depth measurements)

    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        cout << "Error " << returned_state << ", exit program." << endl;
        return EXIT_FAILURE;
    }

    // Capture images and depth

    double **mapa2D;
    mapa2D = new double *[gridWidth];
    for (int i = 0; i < gridWidth; i++) {
        mapa2D[i] = new double[gridHeight];
        for (int j = 0; j < gridHeight; j++) {
            mapa2D[i][j] = 0;
        }
    }

    double **matrix_depth;
    matrix_depth = new double *[gridWidth];
    for (int i = 0; i < gridWidth; i++) {
        matrix_depth[i] = new double[gridHeight];
        for (int j = 0; j < gridHeight; j++) {
            matrix_depth[i][j] = -10000;
        }
    }
    double vyska_od_zeme = 235;
    double posun_os_x = 35;


    int i = 0;
    //sl::Mat image, depth, point_cloud;
    sl::Mat point_cloud;

    sl::Mat image;
    zed.retrieveImage(image, VIEW::LEFT);

    cv::Mat obrazok(gridHeight / 2 *multiplier, gridWidth *multiplier, CV_8UC3);

    while (true) {
        // A new image is available if grab() returns ERROR_CODE::SUCCESS
        if (zed.grab() == ERROR_CODE::SUCCESS) {
            // Retrieve left image
            //zed.retrieveImage(image, VIEW::LEFT);
            // Retrieve depth map. Depth is aligned on the left image
            //zed.retrieveMeasure(depth, MEASURE::DEPTH);
            // Retrieve colored point cloud. Point cloud is aligned on the left image.
            zed.retrieveMeasure(point_cloud, MEASURE::XYZRGBA);

	    static int retrieve_counter = 0;
	    //printf("retrieved %d\n", retrieve_counter++);

            for (int i = 0; i < gridWidth; i++) {
                for (int j = 0; j < gridHeight; j++) {
                    mapa2D[i][j] = 0;
                }
            }
            for (int i = 0; i < gridWidth; i++) {
                for (int j = 0; j < gridHeight; j++) {
                    matrix_depth[i][j] = -10000;
                }
            }

            // hladam Z plochy pred robotom
            for (int x = 0; x < image.getWidth(); x++) {
		//printf("x=%d\n", x);
                for (int y = 0; y < image.getHeight(); y++) {
                    sl::float4 point_cloud_value;
                    point_cloud.getValue(x, y, &point_cloud_value);
                    if (std::isfinite(-point_cloud_value.y)) {
                        if (-point_cloud_value.y > -1000 && -point_cloud_value.y < 1000) {
                            if (point_cloud_value.x > -6000 && point_cloud_value.x < 6000) {
                                if (point_cloud_value.z > -6000 && point_cloud_value.z < 6000) {
                                    if ((vyska_od_zeme - point_cloud_value.y) / 1000.0 >
                                        mapa2D[gridWidth / 2 + (int) (point_cloud_value.x / (10 * gridSize))]
					      [(int) (point_cloud_value.z / (10 * gridSize))])
                                        mapa2D[gridWidth / 2 + (int) (point_cloud_value.x / (10 * gridSize))][(int) (point_cloud_value.z / (10 * gridSize))] =
                                                (vyska_od_zeme - point_cloud_value.y) / 1000.0;
                                }
                            }
                        }
                    }
                }
            }
	    //printf("ubuntu\n");
            // tu treba skusit vykreslit obrazok
            int multiplier = 5;
            for (int x = 0; x < gridWidth; x++) {
                for (int y = 0; y < gridHeight / 2; y++) {
                    Point a(x * multiplier, (multiplier * gridHeight / 2) - y * multiplier);
                    Point b((x + 1) * multiplier - 1, (multiplier * gridHeight / 2) - (y + 1) * multiplier - 1);
                    if (mapa2D[x][y] > 0)
                        rectangle(obrazok, a, b, Scalar(255, 255 * (1 - mapa2D[x][y]), 255 * (1 - mapa2D[x][y])), FILLED);
                    else
                        rectangle(obrazok, a, b, Scalar(255 * (1 - mapa2D[x][y]), 255 * (1 - mapa2D[x][y]), 255), FILLED);

                }
            }
	    //printf("to show\n");
            imshow("2D projection", obrazok);
	    //printf("showed\n");

            vector<pair<int, int>> need_to_be_checked;

            need_to_be_checked.push_back(make_pair(gridWidth / 2 - 1, gridHeight / 2));
            need_to_be_checked.push_back(make_pair(gridWidth / 2 - 1, gridHeight / 2 - 1));
            need_to_be_checked.push_back(make_pair(gridWidth / 2, gridHeight / 2 - 1));
            need_to_be_checked.push_back(make_pair(gridWidth / 2, gridHeight / 2));

            matrix_depth[gridWidth / 2][gridHeight / 2] = 0;
            matrix_depth[gridWidth / 2][gridHeight / 2 - 1] = 0;
            matrix_depth[gridWidth / 2 - 1][gridHeight / 2] = 0;
            matrix_depth[gridWidth / 2 - 1][gridHeight / 2 - 1] = 0;

            int x;
            int y;
            int newX;
            int newY;

	    //printf("whale\n");
            while (need_to_be_checked.size() > 0) {
	        //printf("sz: %d\n",need_to_be_checked.size());
                x = need_to_be_checked[0].first;
                y = need_to_be_checked[0].second;
                if (x > 0 && x < (gridWidth - 1) && y > 0 && y < (gridHeight - 1)) {
                    for (int i = 0; i < 9; i++) {
                        newX = x - 1 + i % 3;
                        newY = y + 1 - i / 3;
                        if (matrix_depth[newX][newY] < 0) {
                            if (fabs(mapa2D[x][y] - mapa2D[newX][newY]) > prekazka ||
                                fabs(mapa2D[x - (newX - x)][y - (newY - y)] - mapa2D[newX][newY]) > prekazka_postupna) {
                                matrix_depth[newX][newY] = 1;
                            }
			    else matrix_depth[newX][newY] = 0;
                            need_to_be_checked.push_back(make_pair(newX, newY));
                        }
                    }
                    //erase first
                }
	        //printf("erazing it!\n");
                need_to_be_checked.erase(need_to_be_checked.begin());
            }
            // tu treba skusit vykreslit obrazok a ak je spravny tak poslat hlavnemu programu
	    //printf("whaled\n");

            for (int x = 0; x < gridWidth; x++) {
                for (int y = 0; y < gridHeight / 2; y++) {
                    Point a(x * multiplier, multiplier * gridHeight / 2 - y * multiplier);
                    Point b((x + 1) * multiplier - 1, multiplier * gridHeight / 2 - (y + 1) * multiplier - 1);
                    rectangle(obrazok, a, b, Scalar(255, 255 * (1 - matrix_depth[x][y]), 255 * (1 - matrix_depth[x][y])), FILLED);
                }
            }
	    //printf("fored\n");
            imshow("processed", obrazok);
	    //printf("two showed\n");
	    waitKey(1);

        }
    }
    // Close the camera
    zed.close();
    return EXIT_SUCCESS;
}
