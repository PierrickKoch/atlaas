/**
 * cc convert_ascii.c -o convert_ascii; ./convert_ascii in.0000 > pc.txt
 */
#include <stdio.h>

static const size_t VELODYNE_HEADER_LINE = 128;

typedef enum VELODYNE_POINT_STATUS {
    VELODYNE_BAD_POINT,
    VELODYNE_GOOD_POINT
} VELODYNE_POINT_STATUS;

typedef struct VELODYNE_CARTESIAN_POINT {
    VELODYNE_POINT_STATUS status;
    float x, y, z;
    unsigned char intensity;
} VELODYNE_CARTESIAN_POINT;

int dump(FILE* file) {
    char aux[VELODYNE_HEADER_LINE]; /* Auxiliary variable for fscanf. */
    int height, width, maxScanWidth, result, good = 0;
    size_t size;
    VELODYNE_CARTESIAN_POINT point;

    result = fscanf(file, "%s", aux);
    /* Reads the header. */
    result = fscanf(file, "%d %d %d\n", &height, &width, &maxScanWidth);
    if (result != 3)
        return -1;

    for (size = 0; size < width * height; size++) {
        result = fread(&point, sizeof(VELODYNE_CARTESIAN_POINT), 1, file);
        if ((result == 1) && (point.status == VELODYNE_GOOD_POINT)) {
            printf("%f %f %f %i\n", point.x, point.y, point.z, point.intensity);
        }
    }
    /* Closes the file. */
    fclose(file);
    return 0;
}

int main(int argc, char * argv[]) {
    FILE* file = stdin;
    if (argc > 1)
        file = fopen(argv[1], "r");
    if (file == NULL)
      return -1;
    return dump(file);
}
