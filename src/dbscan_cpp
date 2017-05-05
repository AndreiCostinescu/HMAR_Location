/* Copyright 2015 Gagarine Yaikhom (MIT License) */
/* modified by Chen */
/* changed all point_d to point_d and changed l to l*/

#include "dbscan.h"

#define CONTACT

node_t *create_node(unsigned int index)
{
    node_t *n = (node_t *) calloc(1, sizeof(node_t));
    if (n == NULL)
        perror("Failed to allocate node.");
    else {
        n->index = index;
        n->next = NULL;
    }
    return n;
}

int append_at_end(
     unsigned int index,
     epsilon_neighbours_t *en)
{
    node_t *n = create_node(index);
    if (n == NULL) {
        free(en);
        return FAILURE;
    }
    if (en->head == NULL) {
        en->head = n;
        en->tail = n;
    } else {
        en->tail->next = n;
        en->tail = n;
    }
    ++(en->num_members);
    return SUCCESS;
}

epsilon_neighbours_t *get_epsilon_neighbours(
    unsigned int index,
    point_d *points,
    unsigned int num_points,
    double epsilon,
    double (*dist)(point_d *a, point_d *b))
{
    epsilon_neighbours_t *en = (epsilon_neighbours_t *)
        calloc(1, sizeof(epsilon_neighbours_t));
    if (en == NULL) {
        perror("Failed to allocate epsilon neighbours.");
        return en;
    }
    for (unsigned int i = 0; i < num_points; i++) {
        if (i == index)
            continue;
        if (dist(&points[index], &points[i]) > epsilon)
            continue;
        else {
            if (append_at_end(i, en) == FAILURE) {
                destroy_epsilon_neighbours(en);
                en = NULL;
                break;
            }
        }
    }
    return en;
}

void destroy_epsilon_neighbours(epsilon_neighbours_t *en)
{
    if (en) {
        node_t *t, *h = en->head;
        while (h) {
            t = h->next;
            free(h);
            h = t;
        }
        free(en);
    }
}

void dbscan(
    point_d *points,
    unsigned int num_points,
    double epsilon,
    unsigned int minpts,
    double (*dist)(point_d *a, point_d *b))
{
    unsigned int i, l = 0;
    for (i = 0; i < num_points; ++i) {
        if (points[i].l == UNCLASSIFIED) {
            if (expand(i, l, points,
                       num_points, epsilon, minpts,
                       dist) == CORE_POINT)
                ++l;
        }
    }
}

int expand(
    unsigned int index,
    unsigned int l,
    point_d *points,
    unsigned int num_points,
    double epsilon,
    unsigned int minpts,
    double (*dist)(point_d *a, point_d *b))
{
    int return_value = NOT_CORE_POINT;
    epsilon_neighbours_t *seeds =
        get_epsilon_neighbours(index, points,
                               num_points, epsilon,
                               dist);
    if (seeds == NULL)
        return FAILURE;

    if (seeds->num_members < minpts)
        points[index].l = NOISE;
    else {
        points[index].l = l;
        node_t *h = seeds->head;
        while (h) {
            points[h->index].l = l;
            h = h->next;
        }

        h = seeds->head;
        while (h) {
            spread(h->index, seeds, l, points,
                   num_points, epsilon, minpts, dist);
            h = h->next;
        }

        return_value = CORE_POINT;
    }
    destroy_epsilon_neighbours(seeds);
    return return_value;
}

int spread(
    unsigned int index,
    epsilon_neighbours_t *seeds,
    unsigned int l,
    point_d *points,
    unsigned int num_points,
    double epsilon,
    unsigned int minpts,
    double (*dist)(point_d *a, point_d *b))
{
    epsilon_neighbours_t *spread =
        get_epsilon_neighbours(index, points,
                       num_points, epsilon,
                       dist);
    if (spread == NULL)
        return FAILURE;
    if (spread->num_members >= minpts) {
        node_t *n = spread->head;
        point_d *d;
        while (n) {
            d = &points[n->index];
            if (d->l == NOISE ||
                d->l == UNCLASSIFIED) {
                if (d->l == UNCLASSIFIED) {
                    if (append_at_end(n->index, seeds)
                        == FAILURE) {
                        destroy_epsilon_neighbours(spread);
                        return FAILURE;
                    }
                }
                d->l = l;
            }
            n = n->next;
        }
    }

    destroy_epsilon_neighbours(spread);
    return SUCCESS;
}

double euclidean_dist(point_d *a, point_d *b)
{
    return sqrt(
    		pow(a->x - b->x, 2) +
            pow(a->y - b->y, 2) +
            pow(a->z - b->z, 2));
}


// ============================================================================
// dbscan
// ============================================================================

void dbscanCluster(
	double epsilon,
	unsigned int minpts,
	unsigned int num_points,
	point_d *p)
{
	if(num_points)	{ dbscan(p, num_points, epsilon, minpts, euclidean_dist); }
	else 			{ cerr << "NO POINTS FOR CLUSTERING!!!"; }
}

void combineNearCluster(
	vector<point_d> &points_,
	vector<point_d> &locations_,
	vector<int> 	&locations_flag_,
	vector<int> 	contact_)
{
	int num_points 		= points_.size();
	int num_locations 	= locations_.size();
	int num_locations2 	= 0;

	if (num_locations < 1)
	{
		for(int i=0;i<num_points;i++)
			num_locations = max((int)points_[i].l, num_locations);
		num_locations += 1;
		num_locations2 = num_locations;
	}
	else
	{
		for(int i=0;i<num_points;i++)
			num_locations2 = max((int)points_[i].l, num_locations2);
		num_locations2 += 1;
	}

	// calculating the centroid of cluster
	vector<point_d> p_tmp; p_tmp.resize(num_locations2);
	vector<double>	count; count.resize(num_locations2);
	for(int i=0;i<num_points;i++)
	{
		if(points_[i].l >= 0)
		{
			p_tmp[(int)points_[i].l] =
					addPoint(p_tmp[(int)points_[i].l], points_[i]);
			count[(int)points_[i].l] += 1;
		}
	}

	for(int i=0;i<num_locations2;i++)
	{
		p_tmp[i] = multiPoint(p_tmp[i],1/count[i]);
		p_tmp[i].l = UNCLASSIFIED;
	}

	// combine cluster if it is less than 0.1m
	bool limit = false;
	for(int i=0;i<num_locations2;i++)
	{
		for(int j=0;j<num_locations2;j++)
		{
			if(j<=i) continue;

			for(int ii=0;ii<num_points;ii++)
				if(points_[ii].l == i && !limit)
					for(int jj=0;jj<num_points;jj++)
						if(points_[jj].l == j)
							if(l2Norm(minusPoint(points_[ii],points_[jj]))<CLUSTER_LIMIT)
								limit = true;

			if(limit)
			{
				limit = false;

				if(p_tmp[i].l>=0 && p_tmp[j].l>=0)
				{
					int big   = max(p_tmp[i].l, p_tmp[j].l);
					int small = min(p_tmp[i].l, p_tmp[j].l);
					for(int ii=0;ii<num_locations2;ii++)
					{
						if (p_tmp[ii].l == big) { p_tmp[ii].l = small; }
					}
				}
				else if(p_tmp[i].l>=0) { p_tmp[j].l = p_tmp[i].l; }
				else if(p_tmp[j].l>=0) { p_tmp[i].l = p_tmp[j].l; }
				else
				{
					if(i<j)	{ p_tmp[i].l = i; p_tmp[j].l = i; }
					else	{ p_tmp[i].l = j; p_tmp[j].l = j; }
				}
			}
			else
			{
				if(p_tmp[i].l!=(int)i && p_tmp[i].l<0) { p_tmp[i].l = i; }
				if(p_tmp[j].l!=(int)j && p_tmp[j].l<0) { p_tmp[j].l = j; }
			}
		}
		//printf("Location %02d: %02f\n", i, p_tmp[i].l);
	}

	// removing the missing cluster labels
	int c = 1;
	for(int i=1;i<num_locations2;i++)
	{
		if(p_tmp[i].l > p_tmp[i-1].l && p_tmp[i].l == i)
		{
			p_tmp[i].l = c;
			for(int ii=i+1;ii<num_locations2;ii++)
			{
				if(p_tmp[ii].l == i) { p_tmp[ii].l = c; }
			}
			c++;
		}
		//printf("Location %02d: %02d\n", i, p_tmp1[i].l );
	}

	// updating cluster label
	for(int i=0;i<num_points;i++)
	{
		if (points_[i].l >= 0) { points_[i].l = p_tmp[(int)points_[i].l].l; }
		//printf("Location %02d: %02f\n", i, points_[i].l );
	}

#ifdef CONTACT
	if(!contact_.empty())
	{
		// checking contact constraint
		num_locations2 = 0;
		for(int i=0;i<num_points;i++)
		{
			num_locations2 = max((int)points_[i].l, num_locations2);
		}
		num_locations2 += 1;
		reshapeVector(locations_flag_,num_locations2);
		vector<double> c1; c1.resize(num_locations2);
		vector<double> c2; c2.resize(num_locations2);
		for(int i=0;i<num_points;i++)
		{
			if (points_[i].l<0) { continue; }
			if (contact_[i]==1) { c1[points_[i].l]+=1; }
			c2[points_[i].l]+=1;
		}
		for(int i=0;i<num_points;i++)
		{
			if (points_[i].l<0) { continue; }
			if (c1[points_[i].l]/
				c2[points_[i].l] < CONTACT_TRIGGER_RATIO)
			{ continue; }
			locations_flag_[points_[i].l] = 1;
		}
	}
#endif

	// calculate the centroid of combined clusters
	p_tmp.clear(); p_tmp.resize(num_locations2);
	count.clear(); count.resize(num_locations2);

	for(int i=0;i<num_points;i++)
	{
		if(points_[i].l >= 0)
		{
			p_tmp[(int)points_[i].l] =
					addPoint(p_tmp[(int)points_[i].l], points_[i]);
			count[(int)points_[i].l] += 1;
		}
		//printf("Location %02d: %02d %02d\n", i, points_[i].l, p_center[points_[i].l].l );
	}

	for(int i=0;i<p_tmp.size();i++)
	{
		p_tmp[i]	= multiPoint(p_tmp[i],1/count[i]);
		p_tmp[i].l 	= 1.0; // boundary starts with 1.0 as no error and goes to zero for large distance boundary
		count[i]	= UNCLASSIFIED;
		//printf("Location %02d: %+.4f %+.4f %+.4f %d\n", i, p_center[i].x, p_center[i].y, p_center[i].z, p_center[i].l);
	}

	locations_ = p_tmp;
}


int clustering(
	vector<point_d> &points_,
	double epsilon,
	unsigned int minpts)
{
	int num_points;
	point_d *points_array;
	num_points = points_.size();
	points_array = Calloc(point_d, num_points);
	vector2array(points_, points_array);
	dbscanCluster(epsilon, minpts, num_points, points_array);
	printer(13);
	reshapeVector(points_, num_points);
	array2vector(points_array, num_points, points_);
	return EXIT_SUCCESS;
}
