//@tian xu
//basic implementation of voroni implementation in real time, calculations done in openCV, this should run as is with right version of opencv

#include "Greenhouse.h"
#include <math.h>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/legacy/legacy.hpp>
#include "opencv2/highgui/highgui.hpp"

using namespace std;

//#define DEBUG2
//#define VERBOSE
//#define COLORFILL


template <class T>
inline std::string to_string (const T& t)
{
    std::stringstream ss;
    ss << t;
    return ss.str();
}


class VoroniPoint : public Image {
public:
    Vect velocity;
    
    VoroniPoint() : Image("/Users/XuXo/Desktop/greenhouse_opencv/ball.png") {
        
		SetHeightUniformly(1.0);
        SetWidthUniformly(1.0);
        
		Vect feld_loc = Feld()-> Loc ();
        float64 width = Feld()-> Width() / 2.0 - 2;
        float64 right = feld_loc.Dot(Feld()-> Over()) + width;
        float64 left = feld_loc.Dot(Feld()-> Over()) - width;
        
        float64 height = Feld()-> Height() / 2.0 - 2;
        float64 top = feld_loc.Dot (Feld ()-> Up()) + height;
        float64 bottom = feld_loc.Dot (Feld()-> Up()) - height;
        
		//SetHeightUniformly(height);
		//SetWidthUniformly(width);
        
        //SetTranslationHard(Vect(Random(left, right), Random(bottom, top), 0.0));
        TranslationAnimateLinear (4);
        SetTranslationHard(Vect(-10,-10, 0.0));
        SlapOnFeld();
        
        
        float a = Random(-5,5);
        float b = Random(-5,5);
		velocity = Vect(a, b, 0.0);
        IncTranslationHard(Vect(velocity));
    }
    
    void Travail() {
        TranslationAnimateLinear (.2);
        IncTranslation(velocity);
        Bounce();
    }
    
protected:
    
    void Bounce() {
        
        Vect feld_loc = Feld()-> Loc ();
        float64 width = (Feld()-> Width() / 2.0) - 2;
        float64 right = feld_loc.Dot(Feld()-> Over()) + width;
        float64 left = feld_loc.Dot(Feld()-> Over()) - width;
        
        float64 height = (Feld()-> Height() / 2.0) - 2;
        float64 top = feld_loc.Dot (Feld ()-> Up()) + height;
        float64 bottom = feld_loc.Dot (Feld()-> Up()) - height;
        
        /*
         cout<<"left edge is"<<left<<endl; //-166
         cout<<"right edge is"<<right<<endl; //166
         cout<<"top edge is"<<top<<endl; //103
         cout<<"bottom edge is"<<bottom<<endl; //-103
         */
        
        if(Translation().x > right)
        {
            velocity.x = -Abs(velocity.x);
        }
        else if(Translation().x < left)
        {
            velocity.x = Abs(velocity.x);
        }
        else if(Translation().y > top)
        {
            velocity.y = -Abs(velocity.y);
        }
        else if(Translation().y < bottom)
        {
            velocity.y = Abs(velocity.y);
        }
    }
};



class VoroniTessellation : public Thing
{
public:
    
    //Trove <Vect> opengl_lines;
	vector<Trove <Vect> > opengl_facets;
    float64 desired_rotation;
    
    Trove <Vect> opengl_lines;
    Trove <Vect> opengl_liness;
    
    float greenhouse_width;     //this is half the screen, ie center being origin
	float greenhouse_height;
    float greenhouse_radius;	//really a half diagonal in this context
    
	float blu = 1;				//for shading the polygons if we're so interested
    
	float left, right, top, bottom;
    
    float cv_window_width;		//this is the whole screen, ie center being top left corner
    float cv_window_height;
    
    std::map<string, VoroniPoint*> moving_points;
    int num_points;
    
    //a vector of vectors of points for each polygon to be drawn in successive order.  This vector will be cleared at each step
    vector<vector<std::pair<float, float> > > voroni_graph;		//deprecated
    
    
    VoroniTessellation (std::map<string, VoroniPoint*> points) : Thing ()
    {
        Text *t = new Text ("Voroni Tessellation with Oblong Greenhouse");
        t->SetTextColor(Color(0.8117, 0.7843, 0.7098));
        t -> SetFontSize (Feld () -> Height () / 40.0);
        t -> SlapOnFeld ();
        t->IncTranslation(Vect(-100,100,0));
        //http://greenhouse.oblong.com/reference/color_methods.html#qr_setalpha
        
        SlapOnFeld();
		Vect feld_loc = Feld()-> Loc ();
        greenhouse_width = ((float)Feld()-> Width() / 2) - 2;
        greenhouse_height = ((float)Feld()-> Height() / 2) - 2;
        
		right = feld_loc.Dot(Feld()-> Over()) + greenhouse_width;
        left = feld_loc.Dot(Feld()-> Over()) - greenhouse_width;
        
        top = feld_loc.Dot (Feld ()-> Up()) + greenhouse_height;
        bottom = feld_loc.Dot (Feld()-> Up()) - greenhouse_height;
        
        #ifdef DEBUG2
        cout<<"\ngreenhouse window dimensions "<<greenhouse_width<<", "<<greenhouse_height<<endl;  // gives (166, 133)
        #endif
        
		moving_points = points;
        num_points = moving_points.size();
    }
    
    void Travail () {
    	voroniInit();
    }
    
    
    
	//handing off to openCV for the main calculation
    void voroniInit() {
        
		//debug statements should be used sparingly at this point since this affects the application framerate..
        
		//replicate the opencv display to be of the same dimensions
		double ratio = (float)greenhouse_height/greenhouse_width;
        cv_window_width = 500;
        cv_window_height = (500*ratio);
        
		greenhouse_radius = sqrt ((cv_window_width/2 * cv_window_width/2) + (cv_window_height/2  * cv_window_height/2 ));
        
        //subdiv2D extends subdiv2d_fields which is of the following form
		/*
         #define CV_SUBDIV2D_FIELDS()       \
         CV_GRAPH_FIELDS()                  \
         int  quad_edges;                   \
         int  is_geometry_valid;			\
         CvSubdiv2DEdge recent_edge;		\
         CvPoint2D32f  topleft;             \
         CvPoint2D32f  bottomright;
         */
		char win[] = "source";
		int i;
		CvRect rect = { 0, 0, cv_window_width, cv_window_height };
		CvMemStorage* storage;
        
        CvSubdiv2D* subdiv;
        
        //using old C IplImage right now, will change to cvimage etc later
        IplImage* img;
        CvScalar active_facet_color, delaunay_color, voronoi_color, bkgnd_color;
        
        active_facet_color = CV_RGB( 255, 0, 0 );
        delaunay_color  = CV_RGB( 0,0,0);
        voronoi_color = CV_RGB(0, 180, 0);
        bkgnd_color = CV_RGB(255,255,255);
        
        img = cvCreateImage( cvSize(rect.width,rect.height), 8, 3 );
        cvSet( img, bkgnd_color, 0 );
        
        //cvNamedWindow( win, 1 );
        
        storage = cvCreateMemStorage(0);
        subdiv = voroniCreateStorage( storage, rect );
        
        
        #ifdef VERBOSE
        printf("Delaunay triangulation will now be built iteratively.\n" "To stop the process, press any key\n\n");
        #endif
        
        //now gather the points one by one for voroni bootstrapping, at each call to travail, the positions of the points will be fed to the calc engine to iteratively build up the
		//the delaunay triangulation from which the voroni tessellation is constructed.  For every position in the greenhouse window, its corresponding cv position is calculated and
		//added to the subdivision
        for (int i = 0; i< num_points; i++)
        {
            VoroniPoint* temp = moving_points["point" + to_string(i) ];
            Vect location = (temp -> Loc ());
            float greenhouse_x = location.x;
            float greenhouse_y = location.y;
            
            Vect point(greenhouse_x, greenhouse_y, 0);
            opengl_liness.Append(point);
            
            #ifdef DEBUG2
            cout<<"point"<<to_string(i)<<" currently located at ("<<greenhouse_x<<", "<<greenhouse_y<<")"<<endl;
            #endif
            
			float cv_x = ((greenhouse_x + greenhouse_width)* cv_window_width )/ (2*greenhouse_width) - 5;
			float cv_y = ((greenhouse_y + greenhouse_height)* cv_window_height )/ (2* greenhouse_height) - 5;
            
			//this is sort of cheating to push the points back to the boundary but does help maintain smoothness.  Several issues here, depending on the intensity of the calculation (to be seen)
			//in this->travail(), it affects the frame rate of the bouncing balls (voroniPoints). A longer stay in this method means greater displacement so we could potentially grab a point that's
			//out of range when we transform it to the opencv canvas.  I use a +/- 5 cushion but sometimes that's still not enough.  so change this accordingly
			//ie.
			//maybe use a threshold parameter controlle by TimeSinceLastTravail ()
            
			if(cv_x <= 0 )
                cv_x = 0;
            if(cv_y <= 0 )
                cv_y = 0;
            
            if(cv_x >= cv_window_width)
                cv_x = cv_window_width;
            if(cv_y >= cv_window_height)
                cv_y = cv_window_height;
            
            #ifdef DEBUG2
			cout<<"corresponding opencv dimensions located at ("<<cv_x<<", "<<cv_y<<")"<<endl;
            #endif
            
			//seems easiest to insert points of type float 32f
            CvPoint2D32f fp = cvPoint2D32f( cv_x,cv_y);
            
            //if( cvWaitKey( 100 ) >= 0 )
            //break;
            
            //with the delaunau triangulation (up to these points) finish, we setup the corresponding vornoi tessellation.
            cvSubdivDelaunay2DInsert( subdiv, fp );
            cvCalcSubdivVoronoi2D( subdiv );
            cvSet( img, bkgnd_color, 0 );
        }
        
        cvSet( img, bkgnd_color, 0 );
        
		//navigating the delaunay subdivisons to extract edges and finally their points and send back to greenhouse for plotting
        voroniEdgeTraversal( subdiv, img );
        
    }
    
    
    
    //creates the storage for the subdivision in the form of CvMemStorage
    CvSubdiv2D* voroniCreateStorage( CvMemStorage* storage,CvRect rect )
    {
        CvSubdiv2D* subdiv;
        
        subdiv = cvCreateSubdiv2D( CV_SEQ_KIND_SUBDIV2D, sizeof(*subdiv),
                                  sizeof(CvSubdiv2DPoint),
                                  sizeof(CvQuadEdge2D),
                                  storage );
        cvInitSubdivDelaunay2D( subdiv, rect );
        
        return subdiv;
    }
    
    
    
    //moves about the subdivision one edge at a time to find all adjacent edges
    void voroniEdgeTraversal( CvSubdiv2D* subdiv, IplImage* img )
    {
        CvSeqReader  reader;
        int i, total = subdiv->edges->total;
        int elem_size = subdiv->edges->elem_size;
        
        #ifdef DEBUG2
        cout<<"total is"<<total<<endl;
        cout<<"elem_size is"<<elem_size<<endl;
        #endif DEBUG2
        
        cvCalcSubdivVoronoi2D( subdiv );
        
        cvStartReadSeq( (CvSeq*)(subdiv->edges), &reader, 0 );
        
        for( i = 0; i < total; i++ )
        {
            
			//just for reference on the data structure....
			//CvQuadEdge2D structure
			/*
             #define CV_QUADEDGE2D_FIELDS()     \
             int flags;							\
             struct CvSubdiv2DPoint* pt[4];		\
             CvSubdiv2DEdge  next[4];		//these are the 4 tangent edges, eLnext, eDnest, eRnext, and eOnext, refer to doc for picture
             
             typedef struct CvQuadEdge2D
             {
             CV_QUADEDGE2D_FIELDS()
             }
             CvQuadEdge2D;
             */
            
            CvQuadEdge2D* edge = (CvQuadEdge2D*)(reader.ptr);
            
            if( CV_IS_SET_ELEM( edge ))
            {
                CvSubdiv2DEdge e = (CvSubdiv2DEdge)edge;
                
				//cvSubdiv2dRotateEdge specifies, which of edges of the same quad-edge as the input one to return, one of: 0 - the input edge (e if e is the input edge) 1 - the rotated edge (eRot)
				//2 - the reversed edge (reversed e (in green)) 3 - the reversed rotated edge (reversed eRot (in green))
                
                // left half
                voroniPointTraversal( img, cvSubdiv2DRotateEdge( e, 1 ));
                
                // right half
                voroniPointTraversal( img, cvSubdiv2DRotateEdge( e, 3 ));
            }
            
            CV_NEXT_SEQ_ELEM( elem_size, reader );
        }
    }
    
    
    
    //moves about the points defined by the edge and those of its neighbors
    void voroniPointTraversal( IplImage* img, CvSubdiv2DEdge edge )
    {
		//all points for a particular facet, to be added to our member opengl_facets
		Trove <Vect> opengl_points;
        
        vector<pair<float, float> > adjacent_points;
        CvSubdiv2DEdge t = edge;
        int i, count = 0;
        CvPoint* buf = 0;
        
        //purely a count
        do
        {
            count++;
            t = cvSubdiv2DGetEdge( t, CV_NEXT_AROUND_LEFT );
        } while (t != edge );
        
        buf = (CvPoint*)malloc( count * sizeof(buf[0]));
        
        // gather points
        t = edge;
        for( i = 0; i < count; i++ )
        {
			//a little more about the structure....
			//CvSubdiv2DPoint structure
			/*
             #define CV_SUBDIV2D_POINT_FIELDS()\
             int            flags;      \
             CvSubdiv2DEdge first;      \
             CvPoint2D32f   pt;         \
             int id;
             
             typedef struct CvSubdiv2DPoint
             {
             CV_SUBDIV2D_POINT_FIELDS()
             }
             CvSubdiv2DPoint;
             */
            
            CvSubdiv2DPoint* pt = cvSubdiv2DEdgeOrg( t );				//this is one of two ways to walk around the edges, you could also use cvSubdiv2DEdgeDst
            //and start from there
            if( !pt ) break;
            
			//buf[i] = cvPoint( cvRound(pt->pt.x), cvRound(pt->pt.y));  we don't need buf to hold the points really
            float cv_x = cvRound(pt->pt.x);
            float cv_y = cvRound(pt->pt.y);
            
            #ifdef DEBUG2
            cout<<"opencv dimensions located at ("<<cv_x<<", "<<cv_y<<")"<<endl;
            #endif
            
			//adjacent_points.push_back(make_pair<float, float>(pt.x, pt.y));
            t = cvSubdiv2DGetEdge( t, CV_NEXT_AROUND_LEFT );
            
			//respective positions back in greenhouse, I'm not sure why I don't need to check boundary here but it just works..
			float greenhouse_x = ((cv_x * (greenhouse_width*2) / cv_window_width) - greenhouse_width);
			float greenhouse_y = ((cv_y * (greenhouse_height*2) / cv_window_height) - greenhouse_height);
            
            #ifdef DEBUG2
			cout<<"corresponding greenhouse dimensions located at ("<<greenhouse_x<<", "<<greenhouse_y<<")"<<endl<<endl;
            #endif
            
			Vect neighbor(greenhouse_x, greenhouse_y, 0);
			opengl_points.Append(neighbor);
            //opengl_points cleared at this point
        }
		opengl_facets.push_back(opengl_points);
		//this ends one iteration ready for opengl to plot now
        
		/*
		 //we can shade the regions with native openCV instead of openGL if so interested
         if( i == count )
         {
         CvSubdiv2DPoint* pt = cvSubdiv2DEdgeDst( cvSubdiv2DRotateEdge( edge, 1 ));
         cvFillConvexPoly( img, buf, count, CV_RGB(rand()&255,rand()&255,rand()&255), CV_AA, 0 );
         cvPolyLine( img, &buf, &count, 1, 1, CV_RGB(0,0,0), 1, CV_AA, 0);
         draw_subdiv_point( img, pt->pt, CV_RGB(0,0,0));
         }*/
        //free( buf );
    }
    
    
    virtual void PreDraw () {
        glLineWidth(.75);
    }
    
    
    void DrawSelf () {
        
        #ifdef DEBUG2
        cout<<"number of displayable pieces" <<opengl_facets.size()<<endl;
        #endif
        
		for (int i = 0; i< opengl_facets.size(); i++)
        {
			float centroid_x, centroid_y, centroid;     //this is all used for painting polygons
            float ratio;
			float sum_x, sum_y = 0;
			
            Trove <Vect> facet = opengl_facets[i];
            
			//SetGLColor (Color(0.8431,0.5450,0.4901));
			SetGLColor( Color(0.8431,0.5450,0.4901));       //new york pink..
        
            glBegin(GL_LINES);
            
			//glColor3f(0.0f, 0.0f, 1.0f); //blue color
			//maybe color filling requires another iteration, well it would have to come after the calculation of blu really
			for (int i = 0; i < facet.Count() - 1; i++)
			{
				Vect line_start = facet.Nth (i);
                
                #ifdef COLORFILL
				sum_x += line_start.x;
				sum_y += line_start.y;
                #endif
                
                #ifdef DEBUG2
				cout<<"line start is"<<line_start<<endl;
                #endif
                
				glVertex (line_start);
				Vect line_stop = facet.Nth (i+1);
                
                #ifdef DEBUG2
				cout<<"line stop is"<<line_stop<<endl;
                #endif
                
				glVertex (line_stop);
			}
			glEnd();
            
            #ifdef COLORFILL
			centroid_x = sum_x/facet.Count();
			centroid_y = sum_y/facet.Count();
			centroid = sqrt((centroid_x * centroid_x) + (centroid_y * centroid_y));
			if (centroid < greenhouse_radius)
				blu = centroid/greenhouse_radius;
			else centroid = 1;
            #endif
            
            
            #ifdef COLORFILL
			glColor3f(0.67, 0.847, blu); //blue color
			glBegin(GL_POLYGON);
			for (int i = 0; i < facet.Count() - 1; i++)
			{
				Vect line_start = facet.Nth (i);
				glVertex (line_start);
			}
			glEnd();
            #endif
            //cout<<"\nok new trove"<<endl;
        }
        opengl_facets.clear();
    }
    
    
    void PostDraw () {
        glLineWidth(.5);
    }
	
};


void Setup()
{
    //SetFeldsColor (Color(0, .5, .5));
    //SetFeldsColor(Color(0.3843, 0.5254, .5529));
    SetFeldsColor( Color(0.2784, 0.3725, .4666));       //blue bayoux..
    //SetFeldsColor ();
    
    HideNeedlePoints();
    vector<VoroniPoint> points;
    //for(int i = 0 ; i<20; i++) {
    //VoroniPoint *point = new VoroniPoint();
    //}
    //points.push_back(point);
    
	std::map<string, VoroniPoint*> set_points;
	for(int i = 0; i<50; i++) {
        string name = "point" + to_string(i);
		set_points["point" + to_string(i)] =  new VoroniPoint();
	}
    
    VoroniTessellation *voroni = new VoroniTessellation(set_points);
}
