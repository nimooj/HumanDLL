#include "stdafx.h"
#include "Vertex.h"

Vertex::Vertex() {
	nullFlag = true;
	idx = 0;
	x = 0;
	y = 0;
	z = 0;

	nx = 0;
	ny = 0;
	nz = 0;
}

Vertex::Vertex(float _x, float _y, float _z) {
	nullFlag = false;
	idx = 0;
	x = _x;
	y = _y;
	z = _z;
	nx = 0;
	ny = 0;
	nz = 0;
}

Vertex::Vertex(int i, float _x, float _y, float _z) {
	nullFlag = false;
	idx = i;
	x = _x;
	y = _y;
	z = _z;
	nx = 0;
	ny = 0;
	nz = 0;
}

Vertex::Vertex(int i, Vertex v) {
	nullFlag = false;
	idx = i;
	x = v.x;
	y = v.y;
	z = v.z;
	nx = 0;
	ny = 0;
	nz = 0;
}

Vertex::Vertex(Vertex* v) {
	nullFlag = false;
	idx = v->idx;
	x = v->x;
	y = v->y;
	z = v->z;
	nx = 0;
	ny = 0;
	nz = 0;
}

Vertex::~Vertex() {
	nullFlag = true;
	idx = 0;
	x = 0;
	y = 0;
	z = 0;
}

bool Vertex::isNull() {
	return nullFlag;
}

void Vertex::set() {
	nullFlag = false;
}

void Vertex::set(float _x, float _y, float _z) {
	nullFlag = false;
	x = _x;
	y = _y;
	z = _z;
}

void Vertex::del() {
	nullFlag = true;
	idx = 0;
	x = 0;
	y = 0;
	z = 0;
}

bool Vertex::operator==(const Vertex& v) {
	if (x == v.x && y == v.y && z == v.z)
		return true;
	else
		return false;
}

bool Vertex::operator!=(const Vertex& v) {
	if (x == v.x && y == v.y && z == v.z)
		return false;
	else
		return true;
}

Vertex Vertex::operator+(const Vertex& v) {
	return add(v);
}

Vertex Vertex::operator-(const Vertex& v) {
	return subtract(v);
}

Vertex Vertex::operator*(const Vertex& v) {
	x *= v.x;
	y *= v.y;
	z *= v.z;
	return this;
}

Vertex Vertex::operator+(const float& n) {
	return add(n);
}

Vertex Vertex::operator/(const float& n) {
	return divide(n);
}

Vertex Vertex::operator-(const int& n) {
	x -= n;
	y -= n;
	z -= n;
	return this;
}

Vertex Vertex::operator*(const float& n) {
	x *= n;
	y *= n;
	z *= n;

	return this;
}

float Vertex::distance(Vertex v) {
	return sqrt(pow(x - v.x, 2) + pow(y - v.y, 2) + pow(z - v.z, 2));
}

float Vertex::distanceToLine(Vertex line0, Vertex line1) {
	Vertex x1 = line0;
	Vertex x2 = line1;
	Vertex x2_x1 = x2 - x1;
	Vertex x1_x0 = x1 - *this;
	Vertex s = x2_x1.cross(x1_x0);
	Vertex p = x2_x1;

	float dist = sqrt(pow(s.x, 2) + pow(s.y, 2) + pow(s.z, 2)) / sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2));

	return dist;
}

Vertex Vertex::closest(vector<Vertex> v) {
	float min = 1000000000;
	Vertex closestV;
	for (vector<Vertex>::iterator i = v.begin(); i != v.end(); i++) {
		Vertex v = (*i);
		float dist = distance(v);

		if (dist < min) {
			min = dist;
			closestV = v;
		}
	}
	return closestV;
}

Vertex Vertex::closest(float x1, float y1, float x2, float y2) {
	Vertex n;

	if (x1 == x2) {
		n = Vertex(x1, y, 0);
	}
	else if (y1 == y2) {
		n = Vertex(x, y1, 0);
	}
	else {
		float slope1 = (y1 - y2) / (x1 - x2);
		float intercept1 = y1 - slope1 * x1;

		float slope2 = -1 / slope1;
		float intercept2 = y - slope2 * x;

		float nx = (intercept2 - intercept1) / (slope1 - slope2);
		float ny = (slope1 * intercept1 - slope2 * intercept2) / (slope1 - slope2);

		n = Vertex(nx, ny, 0);
	}

	return n;
}

void Vertex::print() {
	cout << x << ", " << y << ", " << z << endl;
}

Vertex Vertex::subtract(Vertex v) {
	return Vertex(x - v.x, y - v.y, z - v.z);
}

Vertex Vertex::add(Vertex v) {
	return Vertex(x + v.x, y + v.y, z + v.z);
}

Vertex Vertex::add(float i) {
	return Vertex(x + i, y + i, z + i);
}

Vertex Vertex::multiply(float n) {
	return Vertex(n * x, n * y, n * z);
}

Vertex Vertex::divide(float n) {
	if (n == 0) return Vertex();
	return Vertex(x / n, y / n, z / n);
}

Vertex Vertex::cross(Vertex v) {
	float nx = y * v.z - z * v.y;
	float ny = z * v.x - x * v.z;
	float nz = x * v.y - y * v.x;
	return Vertex(nx, ny, nz);
}

Vertex Vertex::dot(Vertex v) {
	return Vertex(x * v.x, y * v.y, z * v.z);
}

Vertex Vertex::normalize() {
	float s = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
	this->x /= s;
	this->y /= s;
	this->z /= s;

	return *this;
}

float Vertex::length() {
	return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}

float Vertex::sum() {
	return x + y + z;
}

void Vertex::rotate(int axis, float degree) {
	float radian = degree * M_PI / 180;
	float orig_x = 0, orig_y = 0, orig_z = 0;

	switch (axis) {
	case Axis_X :
		orig_y = y;
		orig_z = z;
		y = cos(radian) * orig_y - sin(radian) * orig_z;
		z = sin(radian) * orig_y + cos(radian) * orig_z;
		break;

	case Axis_Y :
		orig_x = x;
		orig_z = z;
		x = cos(radian) * orig_x + sin(radian) * orig_z;
		z = -sin(radian) * orig_x + cos(radian) * orig_z;
		break;

	case Axis_Z :
		orig_x = x;
		orig_y = y;
		x = cos(radian) * orig_x - sin(radian) * orig_y;
		y = sin(radian) * orig_x + cos(radian) * orig_y;
		break;

	default:
		break;
	}
}