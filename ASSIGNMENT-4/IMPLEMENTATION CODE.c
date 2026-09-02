#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <limits.h>

#define MAX_NODES 9
#define INF 1000000
#define NUM_ZONES 6
#define ZONE_START 3
#define CACHE_SIZE 17
#define VEHICLES_PER_ROUND 3
#define CAPACITY_PER_VEHICLE 20

/* ---------------- Graph (Weighted, Adjacency List) ---------------- */
typedef struct EdgeNode {
    int dest;
    int weight;
    struct EdgeNode *next;
} EdgeNode;

EdgeNode *graph[MAX_NODES];
char nodeName[MAX_NODES][6] = {"WH1","WH2","WH3","Z1","Z2","Z3","Z4","Z5","Z6"};

void addEdge(int u, int v, int w) {
    EdgeNode *a = malloc(sizeof(EdgeNode));
    a->dest = v; a->weight = w; a->next = graph[u]; graph[u] = a;

    EdgeNode *b = malloc(sizeof(EdgeNode));
    b->dest = u; b->weight = w; b->next = graph[v]; graph[v] = b;
}

/* ---------------- Route Cache (Hash Table) ---------------- */
typedef struct CacheNode {
    int zone;
    int distance;
    int warehouse;
    struct CacheNode *next;
} CacheNode;

CacheNode *routeCache[CACHE_SIZE];
int cacheHits = 0, cacheMisses = 0;

int hashZone(int zone) { return zone % CACHE_SIZE; }

int cacheLookup(int zone, int *distOut, int *whOut) {
    CacheNode *p = routeCache[hashZone(zone)];
    while (p) {
        if (p->zone == zone) {
            *distOut = p->distance;
            *whOut = p->warehouse;
            return 1;
        }
        p = p->next;
    }
    return 0;
}

void cacheInsert(int zone, int distance, int warehouse) {
    CacheNode *n = malloc(sizeof(CacheNode));
    n->zone = zone;
    n->distance = distance;
    n->warehouse = warehouse;
    n->next = routeCache[hashZone(zone)];
    routeCache[hashZone(zone)] = n;
}

/* ---------------- Dijkstra (multi-source: 3 warehouses) ---------------- */
int dist[MAX_NODES], src[MAX_NODES], visited[MAX_NODES];

void dijkstraFromWarehouses(void) {
    for (int i = 0; i < MAX_NODES; i++) {
        dist[i] = INF;
        visited[i] = 0;
        src[i] = -1;
    }

    for (int w = 0; w < 3; w++) {
        dist[w] = 0;
        src[w] = w;
    }

    for (int iter = 0; iter < MAX_NODES; iter++) {
        int u = -1, best = INF;

        for (int i = 0; i < MAX_NODES; i++)
            if (!visited[i] && dist[i] < best) {
                best = dist[i];
                u = i;
            }

        if (u == -1) break;
        visited[u] = 1;

        for (EdgeNode *e = graph[u]; e; e = e->next) {
            if (!visited[e->dest] && dist[u] + e->weight < dist[e->dest]) {
                dist[e->dest] = dist[u] + e->weight;
                src[e->dest] = src[u];
            }
        }
    }
}

int getRouteDistance(int zone, int *warehouseOut) {
    int d, wh;

    if (cacheLookup(zone, &d, &wh)) {
        cacheHits++;
        *warehouseOut = wh;
        return d;
    }

    cacheMisses++;
    d = dist[zone];
    wh = src[zone];
    cacheInsert(zone, d, wh);
    *warehouseOut = wh;
    return d;
}

/* ---------------- Delivery Request & Priority Queue (Max-Heap) ---------------- */
typedef struct {
    char zoneName[6];
    int zoneId;
    int demand;
    int urgency;       /* 1-10 */
    int distance;      /* km, from route cache */
    int waitingRounds; /* aging counter, prevents starvation */
    double score;
} Request;

Request heap[NUM_ZONES + 5];
int heapSize = 0;

void swapReq(Request *a, Request *b) {
    Request t = *a;
    *a = *b;
    *b = t;
}

void heapUp(int i) {
    while (i > 0) {
        int p = (i - 1) / 2;
        if (heap[p].score >= heap[i].score) break;
        swapReq(&heap[p], &heap[i]);
        i = p;
    }
}

void heapPush(Request r) {
    heap[heapSize] = r;
    heapUp(heapSize);
    heapSize++;
}

void heapDown(int i) {
    while (1) {
        int l = 2 * i + 1, r = 2 * i + 2, largest = i;

        if (l < heapSize && heap[l].score > heap[largest].score)
            largest = l;

        if (r < heapSize && heap[r].score > heap[largest].score)
            largest = r;

        if (largest == i) break;

        swapReq(&heap[i], &heap[largest]);
        i = largest;
    }
}

Request heapPop(void) {
    Request top = heap[0];
    heap[0] = heap[--heapSize];
    heapDown(0);
    return top;
}

/* ---------------- Scoring strategies ---------------- */
double urgencyFirstScore(Request r) {
    return (double)r.urgency * 10.0 + r.waitingRounds * 1.5;
}

double hybridScore(Request r, int maxDemand, int maxDistance) {
    double normDemand = (double)r.demand / maxDemand * 10.0;
    double normDistance = 10.0 - ((double)r.distance / maxDistance * 10.0);

    return 0.4 * r.urgency
         + 0.4 * normDemand
         + 0.2 * normDistance
         + r.waitingRounds * 1.0;
}

/* ---------------- Simulation of dispatch rounds ---------------- */
void runStrategy(const char *label, Request *base, int n, int useHybrid,
                 int maxDemand, int maxDistance) {
    Request pool[NUM_ZONES];

    for (int i = 0; i < n; i++)
        pool[i] = base[i];

    printf("\n=== STRATEGY: %s ===\n", label);
    printf("%-6s %-8s %-8s %-9s %-6s\n",
           "Round", "Zone", "Demand", "Delivered", "Wait");

    int round = 1, remainingZones = n;
    int maxWait = 0;

    while (remainingZones > 0 && round <= 6) {
        heapSize = 0;

        for (int i = 0; i < n; i++) {
            if (pool[i].demand <= 0) continue;

            pool[i].score = useHybrid
                ? hybridScore(pool[i], maxDemand, maxDistance)
                : urgencyFirstScore(pool[i]);

            heapPush(pool[i]);
        }

        int vehiclesLeft = VEHICLES_PER_ROUND;

        while (heapSize > 0 && vehiclesLeft > 0) {
            Request r = heapPop();
            int idx = r.zoneId - ZONE_START;

            int served = pool[idx].demand < CAPACITY_PER_VEHICLE
                       ? pool[idx].demand
                       : CAPACITY_PER_VEHICLE;

            pool[idx].demand -= served;

            printf("%-6d %-8s %-8d %-9d %-6d\n",
                   round, r.zoneName, served, served, r.waitingRounds);

            if (pool[idx].demand <= 0)
                remainingZones--;

            vehiclesLeft--;
        }

        for (int i = 0; i < n; i++) {
            if (pool[i].demand > 0) {
                pool[i].waitingRounds++;
                if (pool[i].waitingRounds > maxWait)
                    maxWait = pool[i].waitingRounds;
            }
        }

        round++;
    }

    printf("Rounds used: %d | Max zone wait: %d round(s)\n",
           round - 1, maxWait);
}

/* ---------------- Performance test: cache + heap scaling ---------------- */
void performanceTest(void) {
    int sizes[] = {100, 500, 1000, 2000, 5000};

    printf("\nRequests\tHeapBuild(sec)\tCacheLookup(sec)\n");

    for (int s = 0; s < 5; s++) {
        clock_t t1 = clock();

        heapSize = 0;

        for (int i = 0; i < sizes[s]; i++) {
            Request r = {
                "ZX",
                ZONE_START + (i % NUM_ZONES),
                (i % 50) + 1,
                (i % 10) + 1,
                (i % 40) + 1,
                0,
                0
            };

            r.score = urgencyFirstScore(r);

            if (heapSize < NUM_ZONES + 5)
                heapPush(r);
        }

        clock_t t2 = clock();

        int d, wh;

        for (int i = 0; i < sizes[s]; i++)
            getRouteDistance(ZONE_START + (i % NUM_ZONES), &wh);

        clock_t t3 = clock();

        (void)d;

        printf("%d\t\t%.6f\t%.6f\n",
               sizes[s],
               (double)(t2 - t1) / CLOCKS_PER_SEC,
               (double)(t3 - t2) / CLOCKS_PER_SEC);
    }
}

int main(void) {
    /* Build warehouse-to-zone transport network */
    addEdge(0, 3, 5);
    addEdge(0, 4, 8);
    addEdge(0, 5, 12);

    addEdge(1, 4, 4);
    addEdge(1, 5, 6);
    addEdge(1, 6, 9);

    addEdge(2, 6, 5);
    addEdge(2, 7, 7);
    addEdge(2, 8, 10);

    addEdge(3, 4, 3);
    addEdge(4, 5, 4);
    addEdge(5, 6, 6);
    addEdge(6, 7, 5);
    addEdge(7, 8, 4);
    addEdge(3, 8, 15);

    dijkstraFromWarehouses();

    printf("============= DISASTER-RELIEF SUPPLY CHAIN ENGINE =============\n");

    printf("\n--- Nearest-Warehouse Route Table (Graph + Dijkstra) ---\n");
    printf("%-6s %-10s %-10s\n", "Zone", "Warehouse", "Distance(km)");

    int wh;

    for (int z = ZONE_START; z < MAX_NODES; z++) {
        int d = getRouteDistance(z, &wh);
        printf("%-6s %-10s %-10d\n", nodeName[z], nodeName[wh], d);
    }

    /* Repeat the same queries to demonstrate route caching */
    for (int z = ZONE_START; z < MAX_NODES; z++)
        getRouteDistance(z, &wh);

    printf("\nRoute cache hits: %d | Route cache misses: %d\n",
           cacheHits, cacheMisses);

    /* Delivery requests for the six disaster zones */
    Request requests[NUM_ZONES];

    int demandArr[NUM_ZONES] = {30, 80, 20, 15, 50, 40};
    int urgencyArr[NUM_ZONES] = {9, 3, 7, 2, 5, 10};

    int maxDemand = 0, maxDistance = 0;

    for (int i = 0; i < NUM_ZONES; i++) {
        int zoneId = ZONE_START + i, w;
        int d = getRouteDistance(zoneId, &w);

        strcpy(requests[i].zoneName, nodeName[zoneId]);
        requests[i].zoneId = zoneId;
        requests[i].demand = demandArr[i];
        requests[i].urgency = urgencyArr[i];
        requests[i].distance = d;
        requests[i].waitingRounds = 0;

        if (demandArr[i] > maxDemand)
            maxDemand = demandArr[i];

        if (d > maxDistance)
            maxDistance = d;
    }

    printf("\n--- Zone Demand / Urgency / Distance Table ---\n");
    printf("%-6s %-8s %-8s %-10s\n",
           "Zone", "Demand", "Urgency", "Distance");

    for (int i = 0; i < NUM_ZONES; i++)
        printf("%-6s %-8d %-8d %-10d\n",
               requests[i].zoneName,
               requests[i].demand,
               requests[i].urgency,
               requests[i].distance);

    runStrategy("URGENCY-FIRST",
                requests, NUM_ZONES, 0, maxDemand, maxDistance);

    runStrategy("HYBRID-WEIGHTED (urgency+demand+distance+aging)",
                requests, NUM_ZONES, 1, maxDemand, maxDistance);

    printf("\n=== PERFORMANCE TEST ===");
    performanceTest();

    printf("\nProgram finished successfully.\n");

    return 0;
}
