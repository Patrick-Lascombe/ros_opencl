kernel void cubedPointcloud(global char* v){
    unsigned int i = get_global_id(0);
    v[i] = v[i] * v[i] * v[i] * 0.15f;
    // The calculation above does not make any sense
    // and does not produce any visual results!
    // You will have to play with byte reading to produce
    // meaningful results.
}

kernel void invertPointcloud(global float* v){
    unsigned int i = get_global_id(0);
    v[i] = -v[i];
}

kernel void minmaxPointcloud(global float* vx, global float* vy, global float* vz, global float* res) {
    unsigned int i = get_global_id(0);

    res[0] = i;
    res[1] = i;
    res[5] = i;
    res[2] = i;
    res[3] = i;
    res[4] = i;

}

kernel void cropPointCloud(global float* vx, global float* vy, global float* vz, global float* minmax, global int* res) {
    unsigned int i = get_global_id(0);
    if((vx[i] > minmax[0]) && (vx[i] < minmax[3]) && (vy[i] > minmax[1]) && (vy[i] < minmax[4]) && (vz[i] > minmax[2]) && (vz[i] < minmax[5])) {
        res[i] = 1;
    } else {
        res[i] = 0;
    }
}

kernel void voxelAssignement(global float* vx, global float* vy, global float* vz, global float* numDivs, global float* bounds, global int* numPoints, global int* firstPoint, global unsigned int* indices, global float* voxel_size) {
    unsigned int i = get_global_id(0);
    int x = (int)((vx[i]/voxel_size[0]) - bounds[0]);
    int y = (int)((vy[i]/voxel_size[0]) - bounds[1]);
    int z = (int)((vz[i]/voxel_size[0]) - bounds[2]);

    unsigned int idx = x + y * numDivs[0] + z * numDivs[0] * numDivs[1];
    unsigned int pointsInVox = numPoints[idx] + 1;

    if(pointsInVox == 1) {
        firstPoint[idx] = i;
    }

    numPoints[idx] = pointsInVox;
    indices[i] = idx;
}

kernel void closerLaserScan(global float* v){
    unsigned int i = get_global_id(0);
    v[i] = v[i] * v[i] * 0.1f;
}

kernel void closerLaserScanDouble(global double* v){
    unsigned int i = get_global_id(0);
    v[i] = v[i] * v[i] * 0.1f;
}

kernel void grayScale(global char* v){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
    }
}

kernel void grayScaleInt(global int* v){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
    }
}

kernel void frameDiff(global char* v, global char* v2){
    unsigned int i = get_global_id(0);
    if (abs(v[i] - v2[i]) < 20) {
        v[i] = 0;
    }
}

kernel void frameDiffCharInt(global char* v, global int* v2){
    unsigned int i = get_global_id(0);
    if (abs(v[i] - v2[i]) < 20) {
        v[i] = 0;
    }
}

kernel void frameDiffCharFloat(global char* v, global float* v2){
    unsigned int i = get_global_id(0);
    if (abs(v[i] - (int)v2[i]) < 20) {
        v[i] = 0;
    }
}

kernel void frameDiffCharDouble(global char* v, global double* v2){
    unsigned int i = get_global_id(0);
    if (abs(v[i] - (int)v2[i]) < 20) {
        v[i] = 0;
    }
}

kernel void frameDiffIntChar(global int* v, global char* v2){
    unsigned int i = get_global_id(0);
    if (abs(v[i] - (int)v2[i]) < 20) {
        v[i] = 0;
    }
}

kernel void frameDiffIntInt(global int* v, global int* v2){
    unsigned int i = get_global_id(0);
    if (abs(v[i] - (int)v2[i]) < 20) {
        v[i] = 0;
    }
}

kernel void frameDiffIntFloat(global int* v, global float* v2){
    unsigned int i = get_global_id(0);
    if (abs(v[i] - (int)v2[i]) < 20) {
        v[i] = 0;
    }
}

kernel void frameDiffIntDouble(global int* v, global double* v2){
    unsigned int i = get_global_id(0);
    if (abs(v[i] - (int)v2[i]) < 20) {
        v[i] = 0;
    }
}

kernel void frameDiffFloatChar(global float* v, global char* v2){
    unsigned int i = get_global_id(0);
    if (abs((int)(v[i] - v2[i])) < 20) {
        v[i] = 0;
    }
}

kernel void frameDiffFloatInt(global float* v, global int* v2){
    unsigned int i = get_global_id(0);
    if (abs((int)(v[i] - v2[i])) < 20) {
        v[i] = 0;
    }
}

kernel void frameDiffFloatFloat(global float* v, global float* v2){
    unsigned int i = get_global_id(0);
    if (abs((int)(v[i] - v2[i])) < 20) {
        v[i] = 0;
    }
}

kernel void frameDiffFloatDouble(global float* v, global double* v2){
    unsigned int i = get_global_id(0);
    if (abs((int)(v[i] - v2[i])) < 20) {
        v[i] = 0;
    }
}

kernel void frameDiffDoubleChar(global double* v, global char* v2){
    unsigned int i = get_global_id(0);
    if (abs((int)(v[i] - v2[i])) < 20) {
        v[i] = 0;
    }
}

kernel void frameDiffDoubleInt(global double* v, global int* v2){
    unsigned int i = get_global_id(0);
    if (abs((int)(v[i] - v2[i])) < 20) {
        v[i] = 0;
    }
}

kernel void frameDiffDoubleFloat(global double* v, global float* v2){
    unsigned int i = get_global_id(0);
    if (abs((int)(v[i] - v2[i])) < 20) {
        v[i] = 0;
    }
}

kernel void frameDiffDoubleDouble(global double* v, global double* v2){
    unsigned int i = get_global_id(0);
    if (abs((int)(v[i] - v2[i])) < 20) {
        v[i] = 0;
    }
}

kernel void doubleGrayScale(global char* v, global char* v2){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
        v2[i] = v2[i];
        v2[i+1] = v2[i];
        v2[i+2] = v2[i];
    }
}

kernel void doubleGrayScaleCharInt(global char* v, global int* v2){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
        v2[i] = v2[i];
        v2[i+1] = v2[i];
        v2[i+2] = v2[i];
    }
}

kernel void doubleGrayScaleCharFloat(global char* v, global float* v2){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
        v2[i] = v2[i];
        v2[i+1] = v2[i];
        v2[i+2] = v2[i];
    }
}

kernel void doubleGrayScaleCharDouble(global char* v, global double* v2){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
        v2[i] = v2[i];
        v2[i+1] = v2[i];
        v2[i+2] = v2[i];
    }
}

kernel void doubleGrayScaleIntChar(global int* v, global char* v2){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
        v2[i] = v2[i];
        v2[i+1] = v2[i];
        v2[i+2] = v2[i];
    }
}

kernel void doubleGrayScaleIntInt(global int* v, global int* v2){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
        v2[i] = v2[i];
        v2[i+1] = v2[i];
        v2[i+2] = v2[i];
    }
}

kernel void doubleGrayScaleIntFloat(global int* v, global float* v2){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
        v2[i] = v2[i];
        v2[i+1] = v2[i];
        v2[i+2] = v2[i];
    }
}

kernel void doubleGrayScaleIntDouble(global int* v, global double* v2){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
        v2[i] = v2[i];
        v2[i+1] = v2[i];
        v2[i+2] = v2[i];
    }
}

kernel void doubleGrayScaleFloatChar(global float* v, global char* v2){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
        v2[i] = v2[i];
        v2[i+1] = v2[i];
        v2[i+2] = v2[i];
    }
}

kernel void doubleGrayScaleFloatInt(global float* v, global int* v2){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
        v2[i] = v2[i];
        v2[i+1] = v2[i];
        v2[i+2] = v2[i];
    }
}

kernel void doubleGrayScaleFloatFloat(global float* v, global float* v2){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
        v2[i] = v2[i];
        v2[i+1] = v2[i];
        v2[i+2] = v2[i];
    }
}

kernel void doubleGrayScaleFloatDouble(global float* v, global double* v2){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
        v2[i] = v2[i];
        v2[i+1] = v2[i];
        v2[i+2] = v2[i];
    }
}

kernel void doubleGrayScaleDoubleChar(global double* v, global char* v2){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
        v2[i] = v2[i];
        v2[i+1] = v2[i];
        v2[i+2] = v2[i];
    }
}

kernel void doubleGrayScaleDoubleInt(global double* v, global int* v2){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
        v2[i] = v2[i];
        v2[i+1] = v2[i];
        v2[i+2] = v2[i];
    }
}

kernel void doubleGrayScaleDoubleFloat(global double* v, global float* v2){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
        v2[i] = v2[i];
        v2[i+1] = v2[i];
        v2[i+2] = v2[i];
    }
}

kernel void doubleGrayScaleDoubleDouble(global double* v, global double* v2){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
        v2[i] = v2[i];
        v2[i+1] = v2[i];
        v2[i+2] = v2[i];
    }
}

kernel void buffSizeTest(global int* v){
    unsigned int i = get_global_id(0);
    v[i] = 0;
}

kernel void tripleGrayScale(global char* v, global char* v2, global char* v3){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
        v2[i] = v2[i];
        v2[i+1] = v2[i];
        v2[i+2] = v2[i];
        v3[i] = v3[i];
        v3[i+1] = v3[i];
        v3[i+2] = v3[i];
    }
}

kernel void frameDiff3(global char* v, global char* v2, global char* v3){
    unsigned int i = get_global_id(0);
    if (abs(v[i] - v2[i]) < 20) {
        v[i] = 0;
    }
    else if (abs(v[i] - v3[i]) < 20) {
        v[i] = 255;
    }
}
