#include <vector>
using namespace std;

// Returns the index of 'edge' in 'cities' vector of size n
int giveIndex(vector<int> cities, int edge, int n) {
    int i;
    for (i = 0; i < n; i++) {
        if (cities[i] == edge) {
            break;
        }
    }
    return i;
}

inline int partition(int index[], double dist[], int low, int high) {
    double pivot = dist[high]; // pivot
    int i = (low - 1);         // Index of smaller element
    int t;
    double t0;

    for (int j = low; j <= high - 1; j++) {
        // If current element is smaller than the pivot
        if (dist[j] < pivot) {
            i++; // increment index of smaller element
            t = index[j];
            index[j] = index[i];
            index[i] = t;

            t0 = dist[j];
            dist[j] = dist[i];
            dist[i] = t0;
        }
    }
    t = index[high];
    index[high] = index[i + 1];
    index[i + 1] = t;

    t0 = dist[high];
    dist[high] = dist[i + 1];
    dist[i + 1] = t0;

    return (i + 1);
}

// Sorts index[] according to dist[]
void quickSort(int index[], double dist[], int low, int high) {
    if (low < high) {
        /* pi is partitioning index, arr[p] is now
        at right place */
        int pi = partition(index, dist, low, high);

        // Separately sort elements before
        // partition and after partition
        quickSort(index, dist, low, pi - 1);
        quickSort(index, dist, pi + 1, high);
    }
}

inline int Qpartition(vector<double> &dist, int low, int high) {
    double pivot = dist[high]; // pivot
    int i = (low - 1);         // Index of smaller element
    double t0;

    for (int j = low; j <= high - 1; j++) {
        // If current element is smaller than the pivot
        if (dist[j] < pivot) {
            i++; // increment index of smaller element
            t0 = dist[j];
            dist[j] = dist[i];
            dist[i] = t0;
        }
    }

    t0 = dist[high];
    dist[high] = dist[i + 1];
    dist[i + 1] = t0;

    return (i + 1);
}

// Sorts dist[] using QuickSort (for vectors)
void QSort(vector<double> &dist, int low, int high) {
    if (low < high) {
        /* pi is partitioning index, arr[p] is now
        at right place */
        int pi = Qpartition(dist, low, high);

        // Separately sort elements before
        // partition and after partition
        QSort(dist, low, pi - 1);
        QSort(dist, pi + 1, high);
    }
}

// Function to find median of a vector
double findMedian(vector<double> a) {
    // First we sort the array
    int n = a.size();
    QSort(a, 0, n - 1);

    // check for even case
    if (n % 2 != 0) {
        return (double)a[n / 2];
    }

    return (double)(a[(n - 1) / 2] + a[n / 2]) / 2.0;
}