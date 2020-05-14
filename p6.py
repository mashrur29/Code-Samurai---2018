from geopy.distance import geodesic
import heapq
import time, math
t = time.time()
ROAD = 0
UTTARA = 1
METRO = 2
BIKOLPO = 3
kmlheader = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://earth.google.com/kml/2.1\">\n<Document>\n<Placemark>\n<name>route.kml</name>\n<LineString>\n<tessellate>1</tessellate>\n<coordinates>\n"
kmlfooter = "</coordinates>\n</LineString>\n</Placemark>\n</Document>\n</kml>"

class heap:
    def __init__(self):
        self.arr = [None for i in range(10000000)]
        self.size = 0

    def pr(self):
        for i in range(self.size):
            print(self.arr[i],end=" ")

    def insert(self, n):
        self.size += 1
        i = self.size -1
        self.arr[i] = n

        while i != 0 and self.arr[self.parent(i)].dist() > self.arr[i].dist():
            temp = self.arr[self.parent(i)]
            self.arr[self.parent(i)] = self.arr[i]
            self.arr[i] = temp
            i = self.parent(i)

    def pop(self):
        if self.size ==0:
            return None
        if self.size == 1:
            self.size -= 1
            return self.arr[0]

        root = self.arr[0]
        self.arr[0] = self.arr[self.size - 1]
        self.size -= 1
        self.minHeapify(0)
        return root

    def minHeapify(self,i):
        l = 2*i+1
        r = 2*i+2
        smallest = i
        if l < self.size and self.arr[l].dist() < self.arr[i].dist():
            smallest = l
        if r < self.size and self.arr[r].dist() < self.arr[smallest].dist():
            smallest = r
        if smallest != i:
            temp = self.arr[i]
            self.arr[i] = self.arr[smallest]
            self.arr[smallest] = temp
            self.minHeapify(smallest)


    def parent(self, i):
        return int(math.floor((i-1)/2))

class search_node:
    def __init__(self,parent, id, cost_so_far, lat,lng,time_so_far,path_id, mode, end_time, destlat,destlng):
        self.parent = parent
        self.id = id
        self.cost_so_far = cost_so_far
        self.destlat=destlat
        self.destlng=destlng
        self.lat = lat
        self.lng = lng
        self.time_so_far = time_so_far
        self.end_time = end_time
        self.path_id=path_id
        self.mode = mode

    def dist(self):
        return self.cost_so_far + geodesic((self.lng, self.lat), (self.destlng, self.destlat)).kilometers * 5.0


    def pr(self):
        print(self.id,self.cost_so_far,self.time_so_far,self.lat,self.lng)

    def __lt__(self, other):
        if self.dist() == other.dist():
            return self.time_so_far < other.time_so_far
        return self.dist() < other.dist()


kml_init,kml_end = None, None
srclat, srclng, destlat, destlng = 90.364255, 23.828335, 90.390157, 23.758382


nodemap = {}
reverse_nodemap = {}
nodes_so_far = 0
edgemap = dict()
file_roadmap = open("Code Samurai 2019 - Problem Resources/Roadmap-Dhaka.csv").readlines()
for i in range(len(file_roadmap)):
    line = file_roadmap[i]
    tokens = line.replace("\n", "").split(",")
    tokens = tokens[1:len(tokens) - 2]
    if float(tokens[0]) not in nodemap:
        nodemap[float(tokens[0])] = dict()
    if float(tokens[1]) not in nodemap[float(tokens[0])]:
        nodemap[float(tokens[0])][float(tokens[1])] = nodes_so_far
        reverse_nodemap[nodes_so_far] = (float(tokens[0]), float(tokens[1]))
        nodes_so_far += 1

    if float(tokens[len(tokens)-2]) not in nodemap:
        nodemap[float(tokens[len(tokens)-2])] = dict()
    if float(tokens[len(tokens)-1]) not in nodemap[float(tokens[len(tokens)-2])]:
        nodemap[float(tokens[len(tokens)-2])][float(tokens[len(tokens)-1])] = nodes_so_far
        reverse_nodemap[nodes_so_far] = (float(tokens[len(tokens)-2]), float(tokens[len(tokens)-1]))
        nodes_so_far += 1

    u = nodemap[float(tokens[0])][float(tokens[1])]
    v = nodemap[float(tokens[len(tokens) - 2])][float(tokens[len(tokens) - 1])]
    tokens = line.replace("\n", "").split(",")

    if u not in edgemap:
        edgemap[u] = [(v, float(tokens[len(tokens)-1]), i, ROAD )]
    else:
        edgemap[u].append((v, float(tokens[len(tokens)-1]), i, ROAD ))

    if v not in edgemap:
        edgemap[v] = [(u, float(tokens[len(tokens)-1]), i, ROAD )]
    else:
        edgemap[v].append((u, float(tokens[len(tokens)-1]), i, ROAD ))


file_bikolpo = open("Code Samurai 2019 - Problem Resources/Routemap-BikolpoBus.csv").readlines()
for i in range(len(file_bikolpo)):
    line = file_bikolpo[i]
    tokens = line.replace("\n", "").split(",")
    if float(tokens[1]) not in nodemap:
        nodemap[float(tokens[1])] = dict()
    if float(tokens[2]) not in nodemap[float(tokens[1])]:
        nodemap[float(tokens[1])][float(tokens[2])] = nodes_so_far
        reverse_nodemap[nodes_so_far] = (float(tokens[1]), float(tokens[2]))
        nodes_so_far += 1

    if float(tokens[len(tokens)-4]) not in nodemap:
        nodemap[float(tokens[len(tokens)-4])] = dict()
    if float(tokens[len(tokens)-3]) not in nodemap[float(tokens[len(tokens)-4])]:
        nodemap[float(tokens[len(tokens)-4])][float(tokens[len(tokens)-3])] = nodes_so_far
        reverse_nodemap[nodes_so_far] = (float(tokens[len(tokens)-4]), float(tokens[len(tokens)-3]))
        nodes_so_far += 1

    u = nodemap[float(tokens[1])][float(tokens[2])]
    v = nodemap[float(tokens[len(tokens)-4])][float(tokens[len(tokens)-3])]
    w = 0.0
    for j in range(1, len(tokens) - 4):
        if j % 2 == 1:
            w += geodesic((tokens[j+1], tokens[j]), (tokens[j+3], tokens[j+2])).kilometers

    if u not in edgemap:
        edgemap[u] = [(v, w, i, BIKOLPO)]
    else:
        edgemap[u].append((v, w, i, BIKOLPO))

    if v not in edgemap:
        edgemap[v] = [(u, w, i, BIKOLPO )]
    else:
        edgemap[v].append((u, w, i, BIKOLPO))




file_uttara = open("Code Samurai 2019 - Problem Resources/Routemap-UttaraBus.csv").readlines()
for i in range(len(file_uttara)):
    line = file_uttara[i]
    tokens = line.replace("\n", "").split(",")
    if float(tokens[1]) not in nodemap:
        nodemap[float(tokens[1])] = dict()
    if float(tokens[2]) not in nodemap[float(tokens[1])]:
        nodemap[float(tokens[1])][float(tokens[2])] = nodes_so_far
        reverse_nodemap[nodes_so_far] = (float(tokens[1]), float(tokens[2]))
        nodes_so_far += 1

    if float(tokens[len(tokens)-4]) not in nodemap:
        nodemap[float(tokens[len(tokens)-4])] = dict()
    if float(tokens[len(tokens)-3]) not in nodemap[float(tokens[len(tokens)-4])]:
        nodemap[float(tokens[len(tokens)-4])][float(tokens[len(tokens)-3])] = nodes_so_far
        reverse_nodemap[nodes_so_far] = (float(tokens[len(tokens)-4]), float(tokens[len(tokens)-3]))
        nodes_so_far += 1

    u = nodemap[float(tokens[1])][float(tokens[2])]
    v = nodemap[float(tokens[len(tokens) - 4])][float(tokens[len(tokens) - 3])]
    w = 0.0
    for j in range(1, len(tokens) - 4):
        if j % 2 == 1:
            w += geodesic((tokens[j + 1], tokens[j]), (tokens[j + 3], tokens[j + 2])).kilometers

    if u not in edgemap:
        edgemap[u] = [(v, w, i, UTTARA)]
    else:
        edgemap[u].append((v, w, i, UTTARA))

    if v not in edgemap:
        edgemap[v] = [(u, w, i, UTTARA)]
    else:
        edgemap[v].append((u, w, i, UTTARA))


file_metro = open("Code Samurai 2019 - Problem Resources/Routemap-DhakaMetroRail.csv").readlines()
for i in range(len(file_metro)):
    line = file_metro[i]
    tokens = line.replace("\n", "").split(",")
    if float(tokens[1]) not in nodemap:
        nodemap[float(tokens[1])] = dict()
    if float(tokens[2]) not in nodemap[float(tokens[1])]:
        nodemap[float(tokens[1])][float(tokens[2])] = nodes_so_far
        reverse_nodemap[nodes_so_far] = (float(tokens[1]), float(tokens[2]))
        nodes_so_far += 1

    if float(tokens[len(tokens)-4]) not in nodemap:
        nodemap[float(tokens[len(tokens)-4])] = dict()
    if float(tokens[len(tokens)-3]) not in nodemap[float(tokens[len(tokens)-4])]:
        nodemap[float(tokens[len(tokens)-4])][float(tokens[len(tokens)-3])] = nodes_so_far
        reverse_nodemap[nodes_so_far] = (float(tokens[len(tokens)-4]), float(tokens[len(tokens)-3]))
        nodes_so_far += 1

    u = nodemap[float(tokens[1])][float(tokens[2])]
    v = nodemap[float(tokens[len(tokens) - 4])][float(tokens[len(tokens) - 3])]
    w = 0.0
    for j in range(1, len(tokens) - 4):
        if j % 2 == 1:
            w += geodesic((tokens[j + 1], tokens[j]), (tokens[j + 3], tokens[j + 2])).kilometers

    if u not in edgemap:
        edgemap[u] = [(v, w, i, METRO)]
    else:
        edgemap[u].append((v, w, i, METRO))

    if v not in edgemap:
        edgemap[v] = [(u, w, i, METRO)]
    else:
        edgemap[v].append((u, w, i, METRO))


visited_nodemap = [(float("inf"), float("inf")) for i in range(nodes_so_far+10)]

def to_minutes(time):
    time = time.split(" ")
    am_pm = time[1]
    time = time[0].split(":")
    hrs = int(time[0])
    min = int(time[1])
    if am_pm == "PM" and hrs != 12:
        hrs += 12

    if am_pm == "AM" and hrs == 12:
        hrs = 0
    return hrs*60+min


def waiting_time(mode, current_time):
    if mode ==METRO:
        metro_start = 60
        if current_time<metro_start:
            return metro_start-current_time
        return 5-((current_time-metro_start) % 5)
    if mode == BIKOLPO:
        bikolpo_start = 7*60
        if current_time<bikolpo_start:
            return bikolpo_start-current_time
        return 20-((current_time-bikolpo_start) % 20)
    if mode == UTTARA:
        uttara_start = 6*60
        if current_time<uttara_start:
            return uttara_start-current_time
        return 10-((current_time-uttara_start) % 10)
    return 0


'''try:
    t1 = nodemap[srclat][srclng]
except KeyError:
    kml_init = (srclat, srclng)
    cur_dist = float("inf")
    for i in reverse_nodemap:
        lat_temp, lng_temp = reverse_nodemap[i]
        next_dist = geodesic((lng_temp, lat_temp), (kml_init[1], kml_init[0])).kilometers
        if next_dist < cur_dist:
            cur_dist = next_dist
            srclat = lat_temp
            srclng = lng_temp



try:
    t1 = nodemap[destlat][destlng]
except KeyError:
    kml_end = (destlat, destlng)
    cur_dist = float("inf")
    for i in reverse_nodemap:
        lat_temp, lng_temp = reverse_nodemap[i]
        next_dist = geodesic((lng_temp,lat_temp),(kml_end[1],kml_end[0])).kilometers
        if next_dist< cur_dist:
            cur_dist = next_dist
            destlat = lat_temp
            destlng = lng_temp

'''


start_time = "06:45 AM"
end_time = "06:46 PM"


start_time_min = to_minutes(start_time)
end_time_min = to_minutes(end_time)
needed_time = [float('inf') for x in range(0, nodes_so_far+10)]

source = nodemap[srclat][srclng]
visited_nodemap[source] = (0.0,0.0)
destination = nodemap[destlat][destlng]
distance_so_far = float('inf')
destination_node = None

# -----------------------------

class heap_dij:
    def __init__(self):
        self.arr = [None for i in range(10000000)]
        self.size = 0

    def pr(self):
        for i in range(self.size):
            print(self.arr[i], end=" ")

    def insert(self, n):
        self.size += 1
        i = self.size - 1
        self.arr[i] = n

        while i != 0 and self.arr[self.parent(i)].dist_dij() > self.arr[i].dist_dij():
            temp = self.arr[self.parent(i)]
            self.arr[self.parent(i)] = self.arr[i]
            self.arr[i] = temp
            i = self.parent(i)

    def pop(self):
        if self.size == 0:
            return None
        if self.size == 1:
            self.size -= 1
            return self.arr[0]

        root = self.arr[0]
        self.arr[0] = self.arr[self.size - 1]
        self.size -= 1
        self.minHeapify(0)
        return root

    def minHeapify(self, i):
        l = 2 * i + 1
        r = 2 * i + 2
        smallest = i
        if l < self.size and self.arr[l].dist_dij() < self.arr[i].dist_dij():
            smallest = l
        if r < self.size and self.arr[r].dist_dij() < self.arr[smallest].dist_dij():
            smallest = r
        if smallest != i:
            temp = self.arr[i]
            self.arr[i] = self.arr[smallest]
            self.arr[smallest] = temp
            self.minHeapify(smallest)

    def parent(self, i):
        return int(math.floor((i - 1) / 2))



class search_node_dij:
    def __init__(self, id, cost_so_far, lat, lng, time_so_far, destlat, destlng):
        self.id = id
        self.cost_so_far = cost_so_far
        self.destlat=destlat
        self.destlng=destlng
        self.lat = lat
        self.lng = lng
        self.time_so_far = time_so_far
        self.end_time = end_time

    def dist(self):
        return self.cost_so_far + geodesic((self.lng, self.lat), (self.destlng, self.destlat)).kilometers * 5.0

    def dist_dij(self):
        return self.cost_so_far + math.ceil(geodesic((self.lng, self.lat), (self.destlng, self.destlat)).kilometers * 3)


    def pr(self):
        print(self.id, self.cost_so_far, self.time_so_far, self.lat, self.lng)

    def __lt__(self, other):
        if self.dist() == other.dist():
            return self.time_so_far < other.time_so_far
        return self.dist() < other.dist()

source_dij = destination
destination_dij = source

def dijks():
    print(source_dij, destination_dij)
    start = search_node_dij(source_dij, 0, srclat, srclng, start_time_min, destlat, destlng)
    q = []
    # heapq.heappush(q, start)
    h = heap_dij()
    h.insert(start)
    nodes = 0
    push_time = 0.0
    while True:
        head = h.pop()
        if head is None:
            break
        current = head

        if(current.cost_so_far > needed_time[current.id]):
            continue

        if current.id == destination_dij:
            # destination_node = current
            break

        for next in edgemap[current.id]:
            next_id = next[0]
            next_cost = current.cost_so_far
            next_time = current.time_so_far
            if next[3] == ROAD:
                # next_cost += next[1] * 20.0
                next_cost += math.ceil(next[1] * 3)
                next_time += math.ceil(next[1] * 3)

            if next[3] == METRO:
                # next_cost += next[1] * 5.0
                next_cost += math.ceil(next[1] * 4)
                next_time += math.ceil(next[1] * 4)

            if next[3] == UTTARA:
                # next_cost += next[1] * 10.0
                next_cost += math.ceil(next[1] * 5)
                next_time += math.ceil(next[1] * 5)

            if next[3] == BIKOLPO:
                # next_cost += next[1] * 7.0
                next_cost += math.ceil(next[1] * 6)
                next_time += math.ceil(next[1] * 6)

            next_cost += waiting_time(next[3], current.time_so_far)
            next_time += waiting_time(next[3], current.time_so_far)

            next_lat, next_lng = reverse_nodemap[next_id]

            if needed_time[next_id] <= next_cost:
                continue

            if next_time + math.ceil(3.0*geodesic((next_lng, next_lat), (current.destlng, current.destlat)).kilometers) > end_time_min:
                continue

            if next_cost <= end_time_min:
                needed_time[next_id] = next_cost
                # temp_t = time.time()
                h.insert(search_node_dij(next_id, next_cost, next_lat, next_lng, next_time, current.destlat, current.destlng))
                # push_time += time.time() - temp_t


dijks()


start = search_node(None, source, 0.0, srclat, srclng,start_time_min,None,None,end_time_min ,destlat, destlng)
q = []
#heapq.heappush(q, start)
h = heap()
h.insert(start)
nodes = 0
push_time = 0.0
while True:
    head = h.pop()
    if head is None:
        break
    current = head
    current.pr()
    if current.id == destination:
        destination_node = current
        break

    for next in edgemap[current.id]:
        next_parent = current
        next_id = next[0]
        next_cost = current.cost_so_far
        next_time = current.time_so_far
        if next[3] == ROAD:
            next_cost += next[1] * 20.0
            next_time += next[1] * 3

        if next[3] == METRO:
            next_cost += next[1] * 5.0
            next_time += next[1] * 4

        if next[3] == UTTARA:
            next_cost += next[1] * 10.0
            next_time += next[1] * 5

        if next[3] == BIKOLPO:
            next_cost += next[1] * 7.0
            next_time += next[1] * 6

        next_time += waiting_time(next[3], current.time_so_far)
        next_lat, next_lng = reverse_nodemap[next_id]
        if next_time <= end_time_min:
            if next_id == destination:
                distance_so_far = min(next_cost, distance_so_far)
            elif next_cost > distance_so_far:
                continue
            t_cost, t_time = visited_nodemap[next_id]

            if (next_cost >= t_cost) and (next_time > t_time):
                continue
            if next_cost<t_cost:
                t_cost, t_time = next_cost,next_time
            elif next_cost == t_cost:
                t_time = min(t_time,next_time)
            if next_time + 3.0*geodesic((next_lng, next_lat), (current.destlng, current.destlat)).kilometers>end_time_min:
                continue
            visited_nodemap[next_id] = t_cost, t_time
            temp_t = time.time()
            h.insert(search_node(next_parent,next_id,next_cost,next_lat,next_lng,next_time,next[2],next[3],current.end_time,current.destlat,current.destlng))
            push_time += time.time() - temp_t
path = []
while True:
    path.append((destination_node.lat, destination_node.lng,destination_node.mode,destination_node.path_id))
    if destination_node.parent is None:
        break
    destination_node = destination_node.parent

path_rev = list(reversed(path))
out_kml = open("route6.kml","w")
out_kml.write(kmlheader)
if kml_init:
    out_kml.write(str(kml_init[0]) + "," + str(kml_init[1]) + ",0\n")

for i in range(len(path_rev)):
    if i == len(path_rev)-1:
        continue
    p, q = path_rev[i], path_rev[i+1]
    if i == 0:
        out_kml.write(str(p[0])+","+str(p[1])+",0\n")
    line = ""

    if q[2]==ROAD:
        line = file_roadmap[q[3]]

    if q[2]==METRO:
        line = file_metro[q[3]]

    if q[2]==UTTARA:
        line = file_uttara[q[3]]

    if q[2]==BIKOLPO:
        line = file_bikolpo[q[3]]
    tokens = line.replace("\n", "").split(",")
    tokens = tokens[1:len(tokens) - 2]
    temp_path = []
    for j in range(len(tokens)):
        if j % 2 == 0:
            temp_path.append((str(tokens[j]), str(tokens[j+1])))
    print("------")
    print(p)
    if float(p[0]) != float(temp_path[0][0]) or float(p[1]) != float(temp_path[0][1]):
        temp_path = list(reversed(temp_path))
    print(temp_path)
    for j in range(1,len(temp_path)):
        out_kml.write(str(temp_path[j][0]) + "," + str(temp_path[j][1]) + ",0\n")
    print(q)
#
kml_init = kml_end

if kml_init:
    out_kml.write(str(kml_init[0]) + "," + str(kml_init[1]) + ",0\n")

out_kml.write(kmlfooter)

print(time.time()-t)
