enum
{
	BIRTH,
	MOVING,
	HIT
};

struct part
{
public:
	part(int p_id)
	{
		id = p_id;
		vel = glm::vec3(0.f,0.f,0.f);
		pos = glm::vec3(0.f,0.f,0.f);
		age = 0.f;
		lifespan = -1.f;
		state = BIRTH;
		gen = 0;
		die = 0;
	}
	int id;
	glm::vec3 vel;
	glm::vec3 pos;
	float age;
	float lifespan;
	glm::vec3 color;
	
	int state;

	int die;
	int gen;
};