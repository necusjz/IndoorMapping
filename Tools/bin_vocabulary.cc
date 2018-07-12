#include "ORBVocabulary.h"

#include <time.h>

using namespace std;

bool load_as_text(indoor_mapping::ORBVocabulary *voc, const std::string infile);
void save_as_binary(indoor_mapping::ORBVocabulary *voc, const std::string outfile);

bool load_as_text(indoor_mapping::ORBVocabulary *voc, const std::string infile) {
    clock_t tStart = clock();
    bool res = voc->loadFromTextFile(infile);
    printf("Loading from text: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
    return res;
}

void save_as_binary(indoor_mapping::ORBVocabulary *voc, const std::string outfile) {
    clock_t tStart = clock();
    voc->saveToBinaryFile(outfile);
    printf("Saving as binary: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

int main(int argc, char **argv) {
    cout << "BoW load/save benchmark" << endl;
    indoor_mapping::ORBVocabulary *voc = new indoor_mapping::ORBVocabulary();

    load_as_text(voc, "Vocabulary/ORBvoc.txt");
    save_as_binary(voc, "Vocabulary/ORBvoc.bin");

    return 0;
}
