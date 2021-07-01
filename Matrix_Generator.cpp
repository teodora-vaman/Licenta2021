#include<iostream>
#include<fstream>

using namespace std;

//data structure for encapsulating an element inside the matrix
struct Index
{
	int i;
	int j;
	int id;

	Index()
	{
		i = j = 0;
		id = -1;
	}

	void set(const int& a, const int& b, const int& new_id)
	{
		i = a;
		j = b;
		id = new_id;
	}

	friend ostream& operator<<(ostream& os, const Index& Aux)
	{
		os << Aux.id << " -> " << Aux.i << " " << Aux.j << endl;
		return os;
	}
};

int main()
{
	//files used for input and output
	ifstream f("plane.txt");
	ofstream g("matrix.txt");
	ofstream h("only_matrix.txt");

	int n;
	int m;
	int nodes = 0;

	f >> n >> m;
	cout << n << " " << m;
	
	// the original image
	int map[300][300];
	//an array to copy the original image only with the seed robots and the starting point
	int matrix[300][300];
	//an array to store the matrix that will be sent to the robots
	float shape_index[2000][4];

	Index seed[4];
	Index start_point;

	//read the image frim the file
	for (int i = 0; i < n; i++)
		for (int j = 0; j < m; j++)
		{
			f >> map[i][j];
			if (map[i][j] == 7)
				map[i][j] = -1;
		}

	for (int i = 0; i < n; i++)
		for (int j = 0; j < m; j++)
		{
			matrix[i][j] = map[i][j];

			if (map[i][j] == -1 || map[i][j] == 4)
			{
				matrix[i][j] = 0;
				nodes++;
			}

			if (map[i][j] == 1 || map[i][j] == 2 || map[i][j] == 3)
				seed[map[i][j] - 1].set(i, j, map[i][j]);

			if (map[i][j] == 4)
				start_point.set(i, j, 4);
		}

	cout << endl;
	cout << nodes << endl;
	cout << start_point.i << " "<< start_point.j << endl;

	int valid_nodes = 0;
	int i;
	int j;
	bool ok = false;
	int vertex = 0;
	int current_node = 0;
	int id = 4;

	//check a the point in the original matrix 
	while (valid_nodes != nodes && current_node < nodes && start_point.i < (n - 2) && start_point.j < (m  - 2) && start_point.i > 1 && start_point.j > 1 && start_point.id < (nodes + 4) && start_point.id >= -1)
	{
		i = start_point.i;
		j = start_point.j;
		vertex = 0;
		
		//cout << start_point << endl;

		//check the 8 elements surrounding the starting point 
		//when one valid element is found update the shape_index
		for (int a = (i - 1); a < (i + 2); a++)
			for (int b = (j - 1); b < (j + 2); b++)
			{
				if (matrix[a][b] != 0 && vertex < 4)
				{
					shape_index[current_node][vertex++] = matrix[a][b]; //the index
					shape_index[current_node][vertex++] = ((a + b) % 2 == (start_point.i + start_point.j) % 2) ? 1.41 : 1; //the distance
				}
			}

		
		if (vertex == 4)
		{
			valid_nodes++;
			matrix[i][j] = start_point.id;
			map[i][j] = start_point.id;
			current_node++;
		}

		int aux_vertex = 0;
		ok = 0;
		for (int i = 1; i < n - 2; i++)
		{
			for (int j = 1; j < m - 2; j++)
			{
				if (map[i][j] == -1)
				{
					//check if point inside map can be the new starting point
					aux_vertex = 0;
					for (int a = (i - 1); a < (i + 2); a++)
						for (int b = (j - 1); b < (j + 2); b++)
						{
							if (matrix[a][b] != 0 && aux_vertex < 4)
							{
								aux_vertex++;
								aux_vertex++;
							}
						}

					//if point has neighbours inside the auxiliary matrix set it as the new starting point
					if (aux_vertex == 4)
					{
						id++;
						start_point.set(i, j, id);
						ok = 1;
						break;

					}
				}
			}

			if (ok == 1)
				break;
		}
	}

	//write the matrix to file
	for (int i = 0; i < nodes; i++)
	{
		g << i + 4 << " : ";
		for (int j = 0; j < 4; j++)
			g << shape_index[i][j] << " ";
		g << endl;
	}

	//write the matrix containing the order of the robots in another file 
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < m; j++)
		{
			h << map[i][j] << " ";
		}
		g << endl;
		h << endl;
	}

	cout << endl;
	system("pause");
	return 0;
}