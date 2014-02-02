/****************************************************************************
**This file is part of the MotorCar QtWayland Compositor, derived from the
**QtWayland QWindow Reference Compositor
**
**
** Copyright (C) 2012 Digia Plc and/or its subsidiary(-ies).
** Contact: http://www.qt-project.org/legal
**
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of Digia Plc and its Subsidiary(-ies) nor the names
**     of its contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
**
****************************************************************************/

#ifndef QWINDOWCOMPOSITOR_H
#define QWINDOWCOMPOSITOR_H

#include "qwaylandcompositor.h"

#include "../../motorcar/src/motorcar.h"
#include "qtwaylandmotorcarscene.h"
#include "qtwaylandmotorcarsurface.h"
#include "defaultdisplaynode.h"
#include "opengldata.h"


#include <QObject>
#include <QTimer>
namespace qtmotorcar{
class QtWaylandMotorcarScene;
class QtWaylandMotorcarCompositor : public QObject, public QWaylandCompositor
{
    Q_OBJECT
public:
    QtWaylandMotorcarCompositor(QOpenGLWindow *window);
    ~QtWaylandMotorcarCompositor();



    motorcar::Display *display() const;
    void setDisplay(motorcar::Display *display);

    OpenGLData *glData() const;
    void setGlData(OpenGLData *glData);



    motorcar::Scene *scene() const;
    void setScene(motorcar::Scene *scene);

    motorcar::WaylandSurfaceNode *getSurfaceNode(QWaylandSurface *surface = NULL) const;

private slots:
    void surfaceDestroyed(QObject *object);
    void surfaceMapped();
    void surfaceUnmapped();
    void surfaceDamaged(const QRect &rect);
    void surfacePosChanged();

    void render();
protected:
    void surfaceDamaged(QWaylandSurface *surface, const QRect &rect);
    void surfaceCreated(QWaylandSurface *surface);

    QWaylandSurface* surfaceAt(const QPointF &point, QPointF *local = 0);

//    GLuint composeSurface(QWaylandSurface *surface);
//    void paintChildren(QWaylandSurface *surface, QWaylandSurface *window);


    bool eventFilter(QObject *obj, QEvent *event);
    QPointF toSurface(QWaylandSurface *surface, const QPointF &point) const;

    void setCursorSurface(QWaylandSurface *surface, int hotspotX, int hotspotY);

    void ensureKeyboardFocusSurface(QWaylandSurface *oldSurface);
//    QImage makeBackgroundImage(const QString &fileName);

private slots:
    void sendExpose();
    void updateCursor();

private:

    motorcar::Scene *m_scene;
    //QList<QWaylandSurface *> m_surfaces;
    OpenGLData *m_glData;
    QTimer m_renderScheduler;

    //Dragging windows around
    QWaylandSurface *m_draggingWindow;
    bool m_dragKeyIsPressed;
    QPointF m_drag_diff;

    //Cursor
    QWaylandSurface *m_cursorSurface;
    int m_cursorHotspotX;
    int m_cursorHotspotY;

    Qt::KeyboardModifiers m_modifiers;

    motorcar::Display *m_display;
    std::map<QWaylandSurface *, motorcar::WaylandSurfaceNode *> m_surfaceMap;

};
}


#endif // QWINDOWCOMPOSITOR_H
