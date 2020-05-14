#include<bits/stdc++.h>
#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#define earthRadiusKm 6371.0
using namespace std;

double pi = 4 * atan(1);
const int maxi = 50000;

double deg2rad(double deg) {
	return (deg * M_PI / 180);
}

double stringToDouble(string word) {
	return atof(word.c_str());
}

pair<double, pair<double, double> > toCartesian(string lat, string lon) {
	double R = 6371;
	double x = R * cos(deg2rad(stringToDouble(lat))) * cos(deg2rad(stringToDouble(lon)));
	double y = R * cos(deg2rad(stringToDouble(lat))) * sin(deg2rad(stringToDouble(lon)));
	double z = R * sin(deg2rad(stringToDouble(lat)));
	return make_pair(x, make_pair(y, z));
}

bool onSegment(string lat1, string lon1, string lat2, string lon2, string latf, string lonf) {
	pair<double, pair<double, double> > tmp1 = toCartesian(lat1, lon1);
	pair<double, pair<double, double> > tmp2 = toCartesian(lat2, lon2);
	pair<double, pair<double, double> > tmpf = toCartesian(latf, lonf);

	double x1 = tmp1.first, y1 = tmp1.second.first, z1 = tmp1.second.second;
	double x2 = tmp2.first, y2 = tmp2.second.first, z2 = tmp2.second.second;
	double xf = tmpf.first, yf = tmpf.second.first, zf = tmpf.second.second;

	double max_x = max(x1, x2);
	double max_y = max(y1, y2);
	double max_z = max(z1, z2);

	double min_x = min(x1, x2);
	double min_y = min(y1, y2);
	double min_z = min(z1, z2);

	if ((xf >= min_x && xf <= max_x) && (yf >= min_y && yf <= max_y) && (zf >= min_z && zf <= max_z))
		return true;
	return false;
}

double orientation(string lat1, string lon1, string lat2, string lon2, string latf, string lonf) {
	double ax = stringToDouble(lat1);
	double ay = stringToDouble(lon1);
	double bx = stringToDouble(lat2);
	double by = stringToDouble(lon2);
	double cx = stringToDouble(latf);
	double cy = stringToDouble(lonf);
	return abs(ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
}

bool onSegmentLatLon(string lat1, string lon1, string lat2, string lon2, string latf, string lonf) {
	double x1 = stringToDouble(lat1);
	double y1 = stringToDouble(lon1);
	double x2 = stringToDouble(lat2);
	double y2 = stringToDouble(lon2);
	double xf = stringToDouble(latf);
	double yf = stringToDouble(lonf);

	double max_x = max(x1, x2);
	double max_y = max(y1, y2);

	double min_x = min(x1, x2);
	double min_y = min(y1, y2);

	if ((xf >= min_x && xf <= max_x) && (yf >= min_y && yf <= max_y)) {
		//cout<<orientation(lat1, lon1, lat2, lon2, latf, lonf)<< " return value"<<endl;
		if (orientation(lat1, lon1, lat2, lon2, latf, lonf) <= 0.00000001) return true;
		return false;
	}
	return false;
}

class CSVRow {
public:
	std::string const& operator[](std::size_t index) const {
		return m_data[index];
	}
	std::size_t size() const {
		return m_data.size();
	}
	void readNextRow(std::istream& str) {
		std::string line;
		std::getline(str, line);

		std::stringstream lineStream(line);
		std::string cell;

		m_data.clear();
		while (std::getline(lineStream, cell, ',')) {
			m_data.push_back(cell);
		}
		// This checks for a trailing comma with no data after it.
		if (!lineStream && cell.empty()) {
			// If there was a trailing comma then add an empty element.
			m_data.push_back("");
		}
	}
private:
	std::vector<std::string> m_data;
};


std::istream& operator>>(std::istream& str, CSVRow& data) {
	data.readNextRow(str);
	return str;
}

//  This function converts radians to decimal degrees
double rad2deg(double rad) {
	return (rad * 180 / M_PI);
}

double getDistanceFromLatLonInKm(double lat1d, double lon1d, double lat2d, double lon2d) {
	double lat1r, lon1r, lat2r, lon2r, u, v;
	swap(lat1d, lon1d);
	swap(lat2d, lon2d);
	lat1r = deg2rad(lat1d);
	lon1r = deg2rad(lon1d);
	lat2r = deg2rad(lat2d);
	lon2r = deg2rad(lon2d);
	u = sin((lat2r - lat1r) / 2);
	v = sin((lon2r - lon1r) / 2);
	return 2.0 * earthRadiusKm * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v));
}

double getDistance(double lat1, double long1, double lat2, double long2) {
	double ret = sqrt((lat1 - lat2) * (lat1 - lat2) + (long1 - long2) * (long1 - long2));
	return ret;
}

std::map<pair<string, string>, int> compress;
std::map<int, pair<string, string> > reverse_compress;
int compressId;
std::vector< pair<pair<int, double>, pair<int, int> > > adj[maxi];
pair<int, int> p[maxi];
std::vector<pair<pair<string, string>, pair<string, string> > > edges;
std::vector<pair<pair<string, string>, pair<int, int> > >  path_temp;
int ids[maxi];
map<pair<int, pair<int, int> >, double> cst;
vector< vector<string> > car_path;
vector< vector<string> > metro_path;
vector< vector<string> > uttara_path;
vector< vector<string> > bikolpo_path;
map<pair<string, string>, string> metro_name, uttara_name, bikolpo_name;
void find_path(int curr) {
	if (p[curr].first != -1)
		find_path(p[curr].first);
	pair<string, string> node = reverse_compress[curr];
	path_temp.push_back(make_pair(make_pair(node.first, node.second), make_pair(p[curr].second, ids[curr])));
}

pair<double, std::vector<pair<pair<string, string>, pair<int, int> > > > dijkstra(int src, int dst1, int dst2) {
	//cout<<reverse_compress[src].first<<" "<<reverse_compress[src].second<<endl;
	priority_queue <pair<double, int> > q;
	double d[maxi];
	for (int i = 0; i < maxi; i++) {
		d[i] = INT_MAX;
		ids[i] = 0;
		p[i] = make_pair(-1, -1);
	}
	q.push(make_pair(0, src));
	d[src] = 0;

	while (!q.empty()) {
		int x = q.top().second;
		int lim = adj[x].size();
		q.pop();
		//cout<<x<<endl;

		for (int i = 0; i < lim; i++) {
			int pp = adj[x][i].first.first;
			double ppw = adj[x][i].first.second;
			int type = adj[x][i].second.first;
			int id = adj[x][i].second.second;
			double cost = 1.0;

			if (type == 0) {
				cost = 20.0;
			}
			else if (type == 1) {
				cost = 5.0;
			}
			else if (type == 2 || type == 3) {
				cost = 7.0;
			}

			double t = d[x] + ppw;


			if (t < d[pp]) {
				p[pp] = make_pair(x, type);
				d[pp] = t;
				ids[pp] = id;
				q.push(make_pair(-1.0 * d[pp], pp));
			}
		}
	}

	path_temp.clear();

	if (d[dst1] < d[dst2])
		find_path(dst1);
	else
		find_path(dst2);

	return make_pair(min(d[dst1], d[dst2]), path_temp);
}


void print_path(vector<pair<pair<string, string>, pair<int, int> > > tmp, string lat1, string lon1, string lat2, string lon2, int case_src, int case_dst) {
	std::ofstream out1("output1.kml");
	string tmp1;
	int n_t = tmp.size();
	out1 << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://earth.google.com/kml/2.1\">\n<Document>\n<Placemark>\n<name>route.kml</name>\n<LineString>\n<tessellate>1</tessellate>\n<coordinates>\n";
	cout << "Problem No: 1" << endl;
	cout << "Source: (" << lat1 << ", " << lon1 << ")" << endl;
	cout << "Destination: (" << lat2 << ", " << lon2 << ")" << endl;
	

	if(case_src == 3) {
		cout<<"Cost: ৳0.00: Walk from Source (" << lat1 << ", " << lon1 << ") to (";
		cout<< tmp[0].first.first << ", " << tmp[0].first.second << ")" << endl;
	}
	if(case_src == 2) {
		double diss = getDistanceFromLatLonInKm(stringToDouble(lat1), stringToDouble(lon1), stringToDouble(tmp[0].first.first), stringToDouble(tmp[0].first.second));
		diss = diss*20.0;
		cout<<"Cost: ৳"<<diss<<": Ride Car from Source (" << lat1 << ", " << lon1 << ") to (";
		cout<< tmp[0].first.first << ", " << tmp[0].first.second << ")" << endl;
	}

	

	for (int i = 0; i < tmp.size() - 1; i++) {
		string start_name = "";
		string endd_name = "";
		int type = tmp[i + 1].second.first;

		int start_id = compress[make_pair(tmp[i].first.first, tmp[i].first.second)];
		int dst_id = compress[make_pair(tmp[i+1].first.first, tmp[i+1].first.second)];

		


		if (type == 1) {
			start_name = metro_name[make_pair(tmp[i].first.first, tmp[i].first.second)];
			endd_name = metro_name[make_pair(tmp[i + 1].first.first, tmp[i + 1].first.second)];
		}
		else if (type == 2) {
			start_name = bikolpo_name[make_pair(tmp[i].first.first, tmp[i].first.second)];
			endd_name = bikolpo_name[make_pair(tmp[i + 1].first.first, tmp[i + 1].first.second)];
		}
		else if (type == 3) {
			start_name = uttara_name[make_pair(tmp[i].first.first, tmp[i].first.second)];
			endd_name = uttara_name[make_pair(tmp[i + 1].first.first, tmp[i + 1].first.second)];
		}

		if(i == 0 && case_src==1) start_name = "Source";
		if(i == n_t-2 && case_dst==1) endd_name = "Destination";
		double cost = 0.0;

		if(type == 0) {
			cost = cst[make_pair(0, make_pair(start_id, dst_id))]*20.0;
			cout<<"Cost: ৳"<<cost<<": ";
			string str1=start_name;
			string str2=endd_name;
			cout<<"Ride Car from "<<str1<<" ("<<tmp[i].first.first<<", "<< tmp[i].first.second<<") ";
			cout<<" to "<<str2<<" ("<<tmp[i+1].first.first<<", "<< tmp[i+1].first.second<<") "<<endl;
		}
		else if(type == 1) {
			cost = cst[make_pair(1, make_pair(start_id, dst_id))]*5.0;
			cout<<"Cost: ৳"<<cost<<": ";
			string str1=start_name;
			string str2=endd_name;
			cout<<"Ride Metro from "<<str1<<" ("<<tmp[i].first.first<<", "<< tmp[i].first.second<<") ";
			cout<<" to "<<str2<<" ("<<tmp[i+1].first.first<<", "<< tmp[i+1].first.second<<") "<<endl;	
		}
		else if(type == 2) {
			cost = cst[make_pair(2, make_pair(start_id, dst_id))]*7.0;
			cout<<"Cost: ৳"<<cost<<": ";
			string str1=start_name;
			string str2=endd_name;
			cout<<"Ride Bikolpo Bus from "<<str1<<" ("<<tmp[i].first.first<<", "<< tmp[i].first.second<<") ";
			cout<<" to "<<str2<<" ("<<tmp[i+1].first.first<<", "<< tmp[i+1].first.second<<") "<<endl;	
		}
		else {
			cost = cst[make_pair(3, make_pair(start_id, dst_id))]*7.0;
			cout<<"Cost: ৳"<<cost<<": ";
			string str1=start_name;
			string str2=endd_name;
			cout<<"Ride Uttara Bus from "<<str1<<" ("<<tmp[i].first.first<<", "<< tmp[i].first.second<<") ";
			cout<<" to "<<str2<<" ("<<tmp[i+1].first.first<<", "<< tmp[i+1].first.second<<") "<<endl;	
		}

		int id = tmp[i + 1].second.second;
		//cout << id << endl;
		if (type == 0) {
			for (int i = 0; i < car_path[id].size() - 1; i += 2) {
				tmp1 = "";
				tmp1 += car_path[id][i];
				tmp1 += ",";
				tmp1 += car_path[id][i + 1];
				tmp1 += ",";
				tmp1 += "0";
				tmp1 += "\n";
				out1 << tmp1;
			}
		}
		else if (type == 1) {
			for (int i = 0; i < metro_path[id].size() - 1; i += 2) {
				tmp1 = "";
				tmp1 += metro_path[id][i];
				tmp1 += ",";
				tmp1 += metro_path[id][i + 1];
				tmp1 += ",";
				tmp1 += "0";
				tmp1 += "\n";
				out1 << tmp1;
			}
		}
		else if (type == 2) {
			for (int i = 0; i < bikolpo_path[id].size() - 1; i += 2) {
				tmp1 = "";
				tmp1 += bikolpo_path[id][i];
				tmp1 += ",";
				tmp1 += bikolpo_path[id][i + 1];
				tmp1 += ",";
				tmp1 += "0";
				tmp1 += "\n";
				out1 << tmp1;
			}
		}
		else {
			for (int i = 0; i < uttara_path[id].size() - 1; i += 2) {
				tmp1 = "";
				tmp1 += uttara_path[id][i];
				tmp1 += ",";
				tmp1 += uttara_path[id][i + 1];
				tmp1 += ",";
				tmp1 += "0";
				tmp1 += "\n";
				out1 << tmp1;
			}
		}
	}
	out1 << "</coordinates>\n</LineString>\n</Placemark>\n</Document>\n</kml>";
	out1.close();


	if(case_dst == 3) {
		cout<<"Cost: ৳0.00: Walk from (" << tmp[n_t-1].first.first << ", " << tmp[n_t-1].first.second << ") to Destination (";
		cout<< lat2 << ", " << lon2 << ")" << endl;
	}
	else if(case_dst == 2) {
		double diss = getDistanceFromLatLonInKm(stringToDouble(tmp[0].first.first), stringToDouble(tmp[0].first.second), stringToDouble(lat2), stringToDouble(lon2));
		diss = diss*20.0;
		cout<<"Cost: ৳"<<diss<<": Ride Car from (" << tmp[n_t-1].first.first << ", " << tmp[n_t-1].first.second << ") to Destination (";
		cout<< lat2 << ", " << lon2 << ")" << endl;
	}
}

void shortest_path(string lat1, string lon1, string lat2, string lon2) {
	int src1, src2, dst1, dst2;
	int case_src = -1, case_dst = -1;

	if (compress.find(make_pair(lat1, lon1)) != compress.end()) {
		src1 = compress[make_pair(lat1, lon1)];
		src2 = src1;
		case_src = 1;
	}
	else {
		bool ok = false;

		for (int i = 0; i < edges.size(); i++) {
			string latt1 = edges[i].first.first;
			string lont1 = edges[i].first.second;
			string latt2 = edges[i].second.first;
			string lont2 = edges[i].second.second;
			if (onSegmentLatLon(latt1, lont1, latt2, lont2, lat1, lon1)) {
				src1 = compress[make_pair(latt1, lont1)];
				src2 = compress[make_pair(latt2, lont2)];
				cout << latt1 << " " << lont1 << " " << latt2 << " " << lont2 << endl;
				ok = true;
				case_src = 2;
				break;
			}
		}

		if (!ok) {
			double minDis = DBL_MAX, minDisId = -1;
			for (int i = 1; i <= compressId; i++) {
				double lat = stringToDouble(reverse_compress[i].first);
				double lon = stringToDouble(reverse_compress[i].second);

				double diss = getDistanceFromLatLonInKm(lat, lon, stringToDouble(lat1), stringToDouble(lon1));
				if (diss < minDis) {
					minDis = diss;
					minDisId = i;
				}
			}
			case_src = 3;
			src1 = src2 = minDisId;
		}
	}


	if (compress.find(make_pair(lat2, lon2)) != compress.end()) {
		dst1 = compress[make_pair(lat2, lon2)];
		dst2 = dst1;
		case_dst = 1;
	}
	else {
		bool ok = false;

		for (int i = 0; i < edges.size(); i++) {
			string latt1 = edges[i].first.first;
			string lont1 = edges[i].first.second;
			string latt2 = edges[i].second.first;
			string lont2 = edges[i].second.second;
			if (onSegmentLatLon(latt1, lont1, latt2, lont2, lat2, lon2)) {
				dst1 = compress[make_pair(latt1, lont1)];
				dst2 = compress[make_pair(latt2, lont2)];
				ok = true;
				case_dst = 2;
				break;
			}
		}

		if (!ok) {
			double minDis = DBL_MAX, minDisId = -1;
			for (int i = 1; i <= compressId; i++) {
				double lat = stringToDouble(reverse_compress[i].first);
				double lon = stringToDouble(reverse_compress[i].second);

				double diss = getDistanceFromLatLonInKm(lat, lon, stringToDouble(lat2), stringToDouble(lon2));
				if (diss < minDis) {
					minDis = diss;
					minDisId = i;
				}
			}
			case_dst = 3;
			dst1 = dst2 = minDisId;
		}
	}


	pair<double, vector<pair<pair<string, string>, pair<int, int> > > > tmp1 = dijkstra(src1, dst1, dst2);
	pair<double, vector<pair<pair<string, string>, pair<int, int> > > > tmp2 = dijkstra(src2, dst1, dst2);


	//cout << case_src << " " << case_dst << endl;
	if (tmp1.first < tmp2.first) {
		print_path(tmp1.second, lat1, lon1, lat2, lon2, case_src, case_dst);
	}
	else {
		print_path(tmp2.second, lat1, lon1, lat2, lon2, case_src, case_dst);
	}
}

void test() {
	string lat1 = "90.404772";
	string lon1 = "23.855136";
	//string lat1 = "90.408772";
	//string lon1 = "23.844125";
	string lat2 = "90.4386";
	string lon2 = "23.79202";
	shortest_path(lat1, lon1, lat2, lon2);
}

void test1() {
	string lat1 = "90.363833";
	string lon1 = "23.834145";
	string lat2 = "90.37836";
	string lon2 = "23.75693";
	shortest_path(lat1, lon1, lat2, lon2);
}

void test2() {
	string lat1 = "90.40023";
	string lon1 = "23.87596";
	string lat2 = "90.367982";
	string lon2 = "23.835966";
	shortest_path(lat1, lon1, lat2, lon2);
}


void test3() {
	string lat1 = "90.36295";
	string lon1 = "23.80874";
	string lat2 = "90.40737";
	string lon2 = "23.73176";
	shortest_path(lat1, lon1, lat2, lon2);
}


void test4() {
	string lat1 = "90.401026";
	string lon1 = "23.876867";
	string lat2 = "90.37866";
	string lon2 = "23.77485";
	shortest_path(lat1, lon1, lat2, lon2);
}

void test5() {
	string lat1 = "90.421608";
	string lon1 = "23.829503";
	string lat2 = "90.37970";
	string lon2 = "23.77603";
	shortest_path(lat1, lon1, lat2, lon2);
}

int main() {
	compressId = 0;
	ifstream file("/home/mashrur/Desktop/BJIT/Code Samurai 2019 - Problem Resources/Roadmap-Dhaka.csv");
	CSVRow row, row1, row2, row3;
	int id;
	while (file >> row) {
		id = car_path.size();
		int n_row = row.size();
		std::vector<string> temp;

		string lat_start = row[1];
		string lon_start = row[2];

		string lat_end = row[n_row - 4];
		string lon_end = row[n_row - 3];

		if (compress[make_pair(lat_start, lon_start)] == 0) {
			compressId = compressId + 1;
			compress[make_pair(lat_start, lon_start)] = compressId;
			reverse_compress[compressId] = make_pair(lat_start, lon_start);
		}

		if (compress[make_pair(lat_end, lon_end)] == 0) {
			compressId = compressId + 1;
			compress[make_pair(lat_end, lon_end)] = compressId;
			reverse_compress[compressId] = make_pair(lat_end, lon_end);
		}

		edges.push_back(make_pair(make_pair(lat_start, lon_start), make_pair(lat_end, lon_end)));

		int u = compress[make_pair(lat_start, lon_start)];
		int v = compress[make_pair(lat_end, lon_end)];
		double w = stringToDouble(row[n_row - 1]);

		cst[make_pair(0, make_pair(u, v))] = w;
		cst[make_pair(0, make_pair(v, u))] = w;

		//cout<<u<<" "<<v<<" "<<w<<endl;
		adj[u].push_back(make_pair(make_pair(v, w), make_pair(0, id)));
		
		adj[v].push_back(make_pair(make_pair(u, w), make_pair(0, id+1)));

		for (int i = 1; i <= row.size() - 3; i++) {
			temp.push_back(row[i]);
		}

		std::vector<string> tempr;


		car_path.push_back(temp);
		reverse(temp.begin(), temp.end());

		for(int i=0; i<temp.size()-1; i+=2) {
			swap(temp[i], temp[i+1]);
		}

		car_path.push_back(temp);
	}


	string src_lat, src_lon, dst_lat, dst_lon;
	//cin >> src_lat >> src_lon >> dst_lat >> dst_lon;
	//shortest_path(src_lat, src_lon, dst_lat, dst_lon);

	test1();
	//cout << compressId << endl;
	//cout << "complete" << endl;
	return 0;
}