
#include <QGLViewer/qglviewer.h>
#include <cppitertools/zip.hpp>

using point3f = std::tuple<float, float, float>;

class Viewer : public QGLViewer
{
    public:
        Viewer(QWidget *parent_, std::shared_ptr<std::vector<std::tuple<float, float, float>>> points_,
                                 std::shared_ptr<std::vector<std::tuple<float, float, float>>> colors_) : QGLViewer(parent_)
        {
            parent = parent_;
            resize(parent->width(), parent->height());
            points = points_;
            colors = colors_;
        };
        ~Viewer(){ saveStateToFile(); };

    protected:
        virtual void draw()
        {
            // drawAxis();
            glColor3f(0.7, 0.7, 0.7);
            drawGrid(5.0, 10);
            glBegin(GL_POINTS);
                for (auto &&[point, color] : iter::zip(*points, *colors))
                {
                    auto &[x,y,z] = point;
                    auto &[r,g,b] = color;
                    glColor3f(r, g, b);
                    glVertex3f(x, y, z);
                }
            glEnd();
            resize(parent->width(), parent->height());  // move to a signal
        };
        virtual void init()
        {
            restoreStateFromFile();
            glDisable(GL_LIGHTING);
            glPointSize(3.0);
            setSceneRadius(100.0);
        };
        virtual void animate(){};
        virtual QString helpString() const { return QString();};

    private:
        std::shared_ptr<std::vector<std::tuple<float, float, float>>> points, colors;
        QWidget *parent;

};
