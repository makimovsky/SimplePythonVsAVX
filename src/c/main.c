#include <stdio.h>
#include <time.h>
#include "CNNModelAVX.h"

int main(int argc, char** argv) {
    //Wczytanie ParamsFile
    size_t sizeNet = sizeof(CNNModelAVXParams);
    CNNModelAVXParams* params = malloc(sizeNet);
    FILE* from = fopen("../../resources/ParamsFile", "rb");
    fread(params, sizeNet, 1, from);
    fclose(from);
    
    /*for(int i=1; i<=346112; i++) {
	if(i <= 32)
		params->convoutputBiases[i-1] = 1.0;
	if(i <= 288)
       		params->convoutputWeights[i-1] = 1.0;
	if(i <= 64)
		params->denseoutputBiases[i-1] = 1.0;
	params->denseoutputWeights[i-1] = 1.0;
	if(i <= 10)
		params->outputlogitsBiases[i-1] = 1.0;
	if(i <= 640)
		params->outputlogitsWeights[i-1] = 1.0;

    }

    for(int i=1; i<=13; i++) {
        int index = 64*32*13*i-1;	
        params->denseoutputWeights[index] = 0.0;
    }
    */

    //Utworzenie siatki
    CNNModelAVXNet* net;
    char* errNet = CNNModelAVXNetCreate(&net, params, 4);
    free(params);
    if (errNet) {
        printf("%s\n", errNet);
        free(errNet);
        exit(1); 
    }

    //Utworzenie silnika
    CNNModelAVXEngine* engine;
    char* errEngine = CNNModelAVXEngineCreate(&engine, net, 4);
    if (errEngine) {
        printf("%s\n", errEngine); 
        free(errEngine); 
        CNNModelAVXNetDestroy(net);
        exit(1); 
    }

    //Wczytanie obrazu i wynik
    float* inputdataData = malloc(sizeof(float)*1*28*28);
    float* outputlogitsData = malloc(sizeof(float)*10*1*1);
    size_t sizeData = sizeof(inputdataData);

    clock_t start, end;
    double t;

    start = clock();
    char filename[35];
    for(int i=0; i<1000; i++) {
	sprintf(filename, "../../resources/images/image%d", i);
	
        FILE* image = fopen(filename, "rb");
        if (image==NULL) {
            printf("Failed to open image file.\n");
            exit(1);
        }
    
        fread(inputdataData, sizeData, 392, image);
        fclose(image);

        CNNModelAVXEngineInference(
            engine,
            inputdataData,
            outputlogitsData
        );
    }
    end = clock();
    t = ((double)(end - start)) / CLOCKS_PER_SEC;

    FILE* ctime = fopen("../../results/ctime.txt", "w");
    fprintf(ctime, "%f\n", t);
    fclose(ctime);
    

    /*FILE* image = fopen("../../resources/images/image0", "rb");
    fread(inputdataData, sizeData, 392, image);
    fclose(image);

    CNNModelAVXEngineInference(
	    engine,
	    inputdataData,
	    outputlogitsData
	);

    printf("\n");
    for (int i = 0; i < 10; i++) {
    	printf("%f\t", outputlogitsData[i]);
    }*/

    //Zwolnienie pamieci
    free(inputdataData);
    free(outputlogitsData);
    CNNModelAVXEngineDestroy(engine);
    CNNModelAVXNetDestroy(net);
}
