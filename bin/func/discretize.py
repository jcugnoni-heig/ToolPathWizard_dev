#from ToolPathWizard_dev.lib.reloading import reloading
import numpy as np
import random

from .Classes import cls_surfaces_grp, cls_surface, cls_operation, cls_curve, cls_point, cls_coordinates, cls_vector, cls_edge, cls_points_of_interest
from .environment import geompy, salome, dataStruct
from .tool_state_fonctions import tool_head_state
from .common_variables import doNotPublishPointInStudy, MACHINING, APPROACH, RETRACT, TRAVEL, WAIT, fdmPerimeter, fdmInfill, milling, tapeLayingAirPulse, tapeLayingLaser, fdmPelletPerimeter, fdmPelletInfill, fdmFiberInfill, fdmFiberPerimeter, generic
from .errors import Unfound_edge, Point_creation_error, Orthogonality_error
from .export import moveType_interpreter
from ..viz.user_com import update_progressBar_and_label
from ..viz.generic_viz import show_item_colorized


debug = True


#@reloading
def discretisation_tool(dataLayerGroup:cls_surfaces_grp, increment:float, UiPBarLay, UiPBarCurves, UiPBarPts, uiLabLay, uiLabCurves, uiLabPts, secuOption:bool, clearOption:bool, securityGeom): #UiProgressBarCurves, UiProgressBarLayers):
    #TODO - ajouter points de retrait, etc dans GEN
    dataLayer:cls_surface
    dataOperation:cls_operation
    dataCurve:cls_curve
    layerCount = 0
    nbLayers = len(dataLayerGroup.surfaceList)
    if nbLayers == 0:
        pass
    update_progressBar_and_label(UiPBarLay, uiLabLay, layerCount, nbLayers, "Layer")
    for dataLayer in dataLayerGroup.surfaceList:
        surfaceGeom, surfaceType = get_geom(dataLayer.surfaceId)
        for dataOperation in dataLayer.operationList:
            curveCount = 0
            nbCurves = len(dataOperation.curveList)
            #print("nbCurves:",nbCurves)
            if nbCurves == 0:
                curveCount += 1
                pass
            startingDistance, endingDistance = __start_and_stop(dataOperation.fabricationMode)
            print(f"On layer {dataLayer.surfaceId}")
            update_progressBar_and_label(UiPBarCurves, uiLabCurves, curveCount, nbCurves, "Curve")
            for dataCurve in dataOperation.curveList:
                #Check longueur minimale
                #print("check")
                if not verify_curve_length(dataCurve, dataOperation.fabricationMode, startingDistance, endingDistance):
                    continue #go to next curve, current curve will not be discretize
                #print("checked")
                #Extraire les edges et afficher si erreur
                edgesList = get_edges(dataCurve.curveId)
                #print("edge list", edgesList)
                pointsOfInterest = points_of_interest(dataOperation.fabricationMode, dataCurve.curveLength, startingDistance, endingDistance, increment, edgesList)
                #print("POI :", pointsOfInterest)
                gen_points(dataCurve, pointsOfInterest, dataOperation.fabricationMode, surfaceGeom, UiPBarPts, secuOption, clearOption, securityGeom, uiLabPts)
                curveCount += 1
                #print("curve Count :", curveCount)
                update_progressBar_and_label(UiPBarCurves, uiLabCurves, curveCount, nbCurves, "Curve")
                #update_progressBar(UiPBarCurves, curveCount/nbCurves*100)   #TODO - Add curve bar
        #update_progressBar(UiPBarLay, layerCount/nbLayers*100)   #TODO - Add layers bar
        layerCount += 1
        update_progressBar_and_label(UiPBarLay, uiLabLay, layerCount, nbLayers, "Layer")
    return


def get_geom(ID:str):
    geom = salome.IDToObject(ID)
    #print(f"SURFACE dataID {ID} geomID {geom.GetEntry()}")
    geomType = str(geom.GetShapeType())
    if geomType == "SHELL" or geomType == "FACE":
        return geom, geomType
    else:
        #TODO - Display error
        return None, None
    

def __start_and_stop(fabricationMode:int):
    return __starting_distance(fabricationMode), __ending_distance(fabricationMode)


def __starting_distance(fabricationMode:int):
    if fabricationMode == fdmPerimeter or fabricationMode == fdmInfill:
        return dataStruct.machineParam.fdmFilament.extrusionDiameter / 2
    elif fabricationMode == fdmPelletPerimeter or fabricationMode == fdmPelletInfill:
        return dataStruct.machineParam.fdmPellet.extrusionDiameter / 2
    elif fabricationMode == milling and dataStruct.machineParam.milling.toolCorrection == True :
        return dataStruct.machineParam.milling.toolDiameter / 2
    elif fabricationMode == tapeLayingLaser:
        return 0 #-dataStruct.machineParam.laserTape.preStartDlf
    #elif fabricationMode == tapeLayingAirPulse:
    #    return dataStruct.machineParam.tapeAirPulse.startDistance
    else :
        return 0


def __ending_distance(fabricationMode:int):
    if fabricationMode == fdmPerimeter or fabricationMode == fdmInfill:
        return dataStruct.machineParam.fdmFilament.extrusionDiameter / 2
    elif fabricationMode == fdmPelletPerimeter or fabricationMode == fdmPelletInfill:
        return dataStruct.machineParam.fdmPellet.extrusionDiameter / 2
    elif fabricationMode == milling and dataStruct.machineParam.milling.toolCorrection == True :
        return dataStruct.machineParam.milling.toolDiameter / 2
    #elif fabricationMode == tapeLayingLaser:
    #    return dataStruct.machineParam.tapeLaser.startDistance
    #elif fabricationMode == tapeLayingAirPulse:
    #    return dataStruct.machineParam.tapeAirPulse.startDistance
    else :
        return 0
    

def verify_curve_length(dataCurve:cls_curve, fabMode:int, startingDistance:float, endingDistance:float):
    #TODO - Si return false : changer la couleur de la courbe et l'afficher + message erreur
    if fabMode==tapeLayingLaser or  fabMode == tapeLayingAirPulse:
        if (dataCurve.curveLength < dataStruct.machineParam.laserTape.minimumTrajectoryLength) or (dataCurve.curveLength < dataStruct.machineParam.airTape.minimumTrajectoryLength):
            if not doNotPublishPointInStudy:
             print(f"Trajectory {dataCurve.curveId} is to short --> {dataCurve.curveLength}mm")
            #raise Curve_to_short(f"Trajectory {dataCurve.curveId} is to short --> {dataCurve.curveLength} mm")
            return False
    if dataCurve.curveLength < dataCurve.increment:
        if not doNotPublishPointInStudy:
            print(f"Trajectory {dataCurve.curveId} is shorter than increment --> {dataCurve.increment}mm")
        return False
    elif dataCurve.curveLength < (startingDistance + endingDistance):
        if not doNotPublishPointInStudy:
            print(f"Trajectory {dataCurve.curveId} is to short. --> start dist + stop dist = {startingDistance + endingDistance}mm")
        return False
    return True
        

#@reloading
def get_edges(curveID:str):
    curveGeom = salome.IDToObject(curveID)
    if str(curveGeom.GetShapeType()) == "EDGE":
        return[cls_edge(curveGeom, geompy.BasicProperties(curveGeom)[0])]
    listEdgeTmp = geompy.ExtractShapes(curveGeom, geompy.ShapeType["EDGE"], False)
    for item in listEdgeTmp:
        if str(item.GetShapeType()) != "EDGE":
            print(f"Item {listEdgeTmp.index(item)}/{len(listEdgeTmp)} is {str(item.GetShapeType())} name is {item.GetName()}")
            print(f"Curve is: {geompy.WhatIs(curveGeom)}")
            print(f"Item source of error is: {geompy.WhatIs(item)}")
            listEdgeTmp.remove(item)
    if listEdgeTmp == []:
        raise Unfound_edge(f"No edges found in curve {curveID}")
    listEdge = []
    for item in listEdgeTmp:
        listEdge.append(cls_edge(item, geompy.BasicProperties(item)[0]))    #[Edge, edge length]
    return listEdge


#@reloading
def points_of_interest(fabMode:int, curveLength:float, startingDistance:float, endingDistance:float, increment:float, edgeList:list):
    pointsOfInterest = cls_points_of_interest(startingDistance, curveLength-endingDistance, edgeList)
    distOnWire = startingDistance
    if fabMode == tapeLayingLaser:
        laserOn = startingDistance + dataStruct.machineParam.laserTape.laserStartingDistance
        cut = pointsOfInterest.stop.distOnWire - dataStruct.machineParam.laserTape.cuttingDistance
        laserOff = pointsOfInterest.stop.distOnWire - dataStruct.machineParam.laserTape.laserStopingDistance
        distOnWire, newIncr = intermediates_points(pointsOfInterest, distOnWire, laserOn, increment, edgeList)
        pointsOfInterest.add_point(laserOn, "laser on", edgeList, newIncr)
        distOnWire, newIncr = intermediates_points(pointsOfInterest, distOnWire, cut, increment, edgeList)
        pointsOfInterest.add_point(cut, "cut", edgeList, newIncr)
        distOnWire, newIncr = intermediates_points(pointsOfInterest, distOnWire, laserOff, increment, edgeList)
        pointsOfInterest.add_point(laserOff, "laser off", edgeList, newIncr)
        distOnWire, newIncr = intermediates_points(pointsOfInterest, distOnWire, pointsOfInterest.stop.distOnWire, increment, edgeList)
        pointsOfInterest.stop.increment = newIncr
    
    elif fabMode == tapeLayingAirPulse:
        cut = pointsOfInterest.stop.distOnWire - dataStruct.machineParam.airTape.cuttingDistance
        distOnWire, newIncr = intermediates_points(pointsOfInterest, distOnWire, cut, increment, edgeList)
        pointsOfInterest.add_point(cut, "cut", edgeList, newIncr)
        distOnWire, newIncr = intermediates_points(pointsOfInterest, distOnWire, pointsOfInterest.stop.distOnWire, increment, edgeList)
        pointsOfInterest.stop.increment = newIncr

    elif fabMode == fdmPelletInfill or fabMode == fdmPelletPerimeter:
        distOnWire, newIncr = intermediates_points(pointsOfInterest, distOnWire, pointsOfInterest.stop.distOnWire, increment, edgeList)
        pointsOfInterest.stop.increment = newIncr
        
    else :
        raise Exception("Add here points of interest for others tools") #TODO
    #TODO - Vérifier si besoin ou si interne à la classe suffit
    #pointsOfInterest.nb = len(pointsOfInterest.op)+1
    #if debug:
    #    print(f"dist:{pointsOfInterest.start.distOnWire}\tdescriptor: {pointsOfInterest.start.descriptor}")
    #    op:cls_points_of_interest.info_point
    #    for op in pointsOfInterest.op:
    #        print(f"dist:{op.distOnWire}\tdescriptor: {op.descriptor}\tincr: {op.increment}")
    #    print(f"dist:{pointsOfInterest.stop.distOnWire}\tdescriptor: {pointsOfInterest.stop.descriptor}")
    #    print(f"Nb points : {pointsOfInterest.nb} Reel: {len(pointsOfInterest.op)+2}")
    #    raise Exception("test liste")
    return pointsOfInterest


def intermediates_points(poi:cls_points_of_interest, distOnWire:float, goalDist:float, increment:float, edgeList:list):
    nbPoints, newIncr = nb_points_and_step(distOnWire, goalDist, increment)
    point:cls_points_of_interest.info_point
    for i in range(nbPoints):
        distOnWire += newIncr
        if i != nbPoints-1: #lastPoint
            poi.add_point(distOnWire, "", edgeList, newIncr)
    return distOnWire, newIncr


def nb_points_and_step(actualPointDistOnCurve:float, goalPointDistOnCurve:float, increment:float):
    dist = goalPointDistOnCurve - actualPointDistOnCurve
    nbPoints = dist/increment
    a = nbPoints - int(nbPoints)
    if a < 0.5:
        nbPoints = int(nbPoints)
    else :
        nbPoints = int (nbPoints) + 1
    newIncr = round(dist/nbPoints, 3)
    return nbPoints, newIncr


#@reloading
def gen_points(curve:cls_curve, PtsOI:cls_points_of_interest, fabricationMode:int, surfaceGeom, UiProgressBar, secuOption:bool, clearOption:bool, securityGeom, uiLabel):
    curveGeom = salome.IDToObject(curve.curveId)
    speed = speed_selector(MACHINING, fabricationMode)
    pointGeom, dataPoint = create_point(curve, curveGeom, PtsOI.start, speed, fabricationMode, surfaceGeom, PtsOI, UiProgressBar, uiLabel) #Point Work
    secuPoint, apPoint = approach_points(curve, dataPoint, secuOption, clearOption, securityGeom, pointGeom, curveGeom, fabricationMode)
    curve.add_point_to_curve(dataPoint)
    publish_vectors(pointGeom, dataPoint)
    for poi in PtsOI.op:
        pointGeom, dataPoint = create_point(curve, curveGeom, poi, speed, fabricationMode, surfaceGeom, PtsOI, UiProgressBar, uiLabel) #Point Work
        curve.add_point_to_curve(dataPoint)
    pointGeom, dataPoint = create_point(curve, curveGeom, PtsOI.stop, "0", fabricationMode, surfaceGeom, PtsOI, UiProgressBar, uiLabel) #Point Work
    curve.add_point_to_curve(dataPoint)
    secuPoint, rePoint = retract_points(curve, dataPoint, secuOption, clearOption, securityGeom, pointGeom, curveGeom, fabricationMode)
    publish_vectors(pointGeom, dataPoint)
    return curve


def speed_selector(moveType:int , fabricationMode:int, stopCondition=False):
    #vitesse selon le mode de fabrication et le type de mouvement
    if stopCondition or moveType == WAIT:
        return '0'
    else:
        if moveType == APPROACH:
            return dataStruct.machineParam.generics.approachSpeed
        elif moveType == RETRACT:
            return dataStruct.machineParam.generics.retractSpeed
        elif moveType == TRAVEL:
            return dataStruct.machineParam.generics.travelSpeed
        else:
            if fabricationMode == fdmPerimeter:
                return dataStruct.machineParam.fdmFilament.perimeterSpeed
            elif fabricationMode == fdmInfill:
                return dataStruct.machineParam.fdmFilament.infillSpeed
            elif fabricationMode == milling :
                return dataStruct.machineParam.milling.toolSpeed
            elif fabricationMode == tapeLayingLaser:
                return dataStruct.machineParam.laserTape.toolSpeed
            elif fabricationMode == tapeLayingAirPulse:
                return dataStruct.machineParam.airTape.toolSpeed
            elif fabricationMode == fdmPelletPerimeter:
                return dataStruct.machineParam.fdmPellet.perimeterSpeed
            elif fabricationMode == fdmPelletInfill:
                return dataStruct.machineParam.fdmPellet.infillSpeed
            else:
                return dataStruct.machineParam.generics.toolSpeed


#@reloading
def create_point(curve:cls_curve, curveGeom, POI:cls_points_of_interest.info_point, speed, fabricationMode:int, surfaceGeom, PtsOI:cls_points_of_interest, UiProgressBar, uiLabel):
    try :
        vertexGeom, faceGeom = point_and_face(POI, surfaceGeom, curve)
        #vertexGeom = point_on_edge(POI.distOnEdge, POI.edgeGeom)
        #faceGeom = get_face(vertexGeom, surfaceGeom)
        vertexEntry, coordinates = add_to_study(curveGeom, vertexGeom, PtsOI.count)
        nVector, tVector = gen_vectors(faceGeom, vertexGeom, POI, vertexEntry)
        toolHeadState = tool_head_state(fabricationMode, MACHINING, POI.increment, curve.curveLength, POI.distOnWire)
        #if cfrDlfVersion and dataStruct.machineParam.laserTape.offset :    #NOTE - Pourrais causer problème lors de la détection de colision. Faire du côté gen traj robotique
        #    coordinates = offset_point(point, dataStruct.machineParam.laserTape.offset)
        point = cls_point(coordinates, vertexEntry, POI.distOnWire, MACHINING, None, nVector, tVector, toolHeadState, speed, POI.stop, POI.increment)
        #curve.add_point_to_curve(point)
        #update_progressBar(UiProgressBar, PtsOI.count/PtsOI.nb*100)
        #check_orthogonality(vertexGeom, point)
        #if not doNotPublishPointInStudy :
        #    update_progressBar_and_label(UiProgressBar, uiLabel, PtsOI.count, PtsOI.nb, "Point")
        update_progressBar_and_label(UiProgressBar, uiLabel, PtsOI.count, PtsOI.nb, "Point")
        PtsOI.count +=1
    except Point_creation_error(f"Failed to create point at {POI.distOnWire}mm on curve {curve.curveId}") or Orthogonality_error:
        pass
    return vertexGeom, point


def point_on_edge(distOnEdge:float, edgeGeom):
    vertexGeom = geompy.MakeVertexOnCurveByLength(edgeGeom, distOnEdge, None)   #vertexGeom = geompy.MakeVertexOnCurve(edgeGeom, )
    return vertexGeom


def add_to_study(curveGeom, vertexGeom, descriptor):
    if doNotPublishPointInStudy:
        entry = "Unpublished"
    else :
        entry = geompy.addToStudyInFather(curveGeom, vertexGeom, f"Point_{descriptor}")
    coordinates = cls_coordinates(geompy.PointCoordinates(vertexGeom))
    return entry, coordinates


def get_face(vertexGeom, surfaceGeom):
    if str(surfaceGeom.GetShapeType()) == "FACE":
        return surfaceGeom
    ntrials=9
    trialstep=1e-3
    trialstepfactor=2
    try:
        faceresult = geompy.GetFaceNearPoint(surfaceGeom, vertexGeom)
    except:
        print("get_face: cannot find face or multiple faces close to point => random search near point activated!")
        for i in range(ntrials):
            trialstep = trialstep * trialstepfactor
            coords = geompy.PointCoordinates(vertexGeom)
            newX = coords[0] + trialstep * random.uniform(-1.0,1.0)
            newY = coords[1] + trialstep * random.uniform(-1.0,1.0)
            newZ = coords[2] + trialstep * random.uniform(-1.0,1.0)
            trialvertex=geompy.MakeVertex(newX, newY, newZ)
            try:
                faceresult = geompy.GetFaceNearPoint(surfaceGeom, trialvertex)
                del trialvertex
                print("Found face at iteration ", i, ", tolerance = ", trialstep  )
                return faceresult
            except:
                pass
        print("get_face: unique face not found near given point, return None")
        return None
    return faceresult 


def point_and_face(POI:cls_points_of_interest.info_point, surfaceGeom, curve:cls_curve):
    #print(f"point_and_face: create point at {POI.distOnWire}mm on curve {curve.curveId}")
    vertexGeom = point_on_edge(POI.distOnEdge, POI.edgeGeom)
    try:
        faceGeom = get_face(vertexGeom, surfaceGeom)
        if faceGeom == None:
            print("Error in get Face: no result returned")
    except RuntimeError:
        print(f"Failed to create point at {POI.distOnWire}mm on curve {curve.curveId}")
        print(f"Attempt by shifting the point by  1e-3mm along edge ")
        POI.distOnEdge = POI.distOnEdge + 1.00E-3
        vertexGeom, faceGeom = point_and_face(POI, surfaceGeom, curve)
    return vertexGeom, faceGeom


#@reloading
def gen_vectors(faceGeom, vertexGeom, POI:cls_points_of_interest.info_point, vertexEntry):
    nVector, normGeom = normal_vector(faceGeom, vertexGeom)
    #tVector, tanGeom = tangent_vector(POI.edgeGeom, POI.distOnWire)
    tVector = alternative_tangent_method(POI.edgeGeom, POI.distOnEdge)
    if check_orthogonality(nVector, tVector, vertexEntry):
        #tVector = alternative_tangent_method(POI.edgeGeom, POI.distOnEdge)
        tVector, tanGeom = tangent_vector(POI.edgeGeom, POI.distOnWire)
        if check_orthogonality(nVector, tVector, vertexEntry):
            print("Tangent vector defect")
            if not doNotPublishPointInStudy:
                show_item_colorized(vertexEntry)
            angle, cosa = vectors_angle(nVector, tVector)
            raise Orthogonality_error(f"The angle between the vectors at point {vertexEntry} is {round(angle,3)}deg.")
        else:
            print(f"Tangent vector corrected {np.single(nVector.vx*tVector.vx + nVector.vy*tVector.vy + nVector.vz*tVector.vz)}")
    return nVector, tVector


def normal_vector(surfaceGeom, vertexGeom):
    vectGeom = geompy.GetNormal(surfaceGeom, vertexGeom)
    dataVector = vector_to_data(vectGeom)
    return dataVector, vectGeom


#@reloading
def tangent_vector(curveGeom, distanceOnCurve:float):
    vectGeom = geompy.MakeTangentOnCurve(curveGeom, distanceOnCurve)
    dataVector = vector_to_data(vectGeom)
    return dataVector, vectGeom


def alternative_tangent_method(curveGeom, distanceOnCurve:float):
    delta = 1E-6
    p1=geompy.MakeVertexOnCurveByLength(curveGeom, distanceOnCurve+delta)
    p2=geompy.MakeVertexOnCurveByLength(curveGeom, distanceOnCurve-delta)
    X1V = np.array(geompy.PointCoordinates(p1))
    X2V = np.array(geompy.PointCoordinates(p2))
    vectorArray = (X1V - X2V) / np.linalg.norm(X1V - X2V)    #compute the tangent vector :  => vt = (X1-X2)/norm(X1-X2)
    delete_geom([p1, p2])
    return cls_vector(vectorArray)


def normalize_vector(vectGeom):
    vertexList = geompy.ExtractShapes(vectGeom, geompy.ShapeType["VERTEX"], False) #Extract vertex (point)
    p0 = np.array(geompy.PointCoordinates(vertexList[0]))
    p1 = np.array(geompy.PointCoordinates(vertexList[1]))
    vectorArray = (p1 - p0) / np.linalg.norm(p1 - p0) #Normalise
    delete_geom(vertexList)
    return vectorArray


def vector_to_data(vectGeom):
    vectorArray = normalize_vector(vectGeom)
    dataVector = cls_vector(vectorArray)
    delete_geom([vectGeom])
    return dataVector


#@reloading
#def check_orthogonality(vertexGeom, point:cls_point):
def check_orthogonality(norm:cls_vector, tan:cls_vector, vertexEntry:str):#, vertexGeom, dataPoint:cls_point):
    angle, cosa = vectors_angle(norm, tan)
    scal = np.single(norm.vx*tan.vx + norm.vy*tan.vy + norm.vz*tan.vz)
    if scal > np.single(1E-4):
    #if cosa > np.single(1E-6):  #Hugo : 1E-4 sur le produit scalaire
        print(f"The angle between the vectors at point {vertexEntry} is {round(angle,3)}deg. Cos = {round(cosa,3)}. Prod scal = {round(norm.vx*tan.vx + norm.vy*tan.vy + norm.vz*tan.vz,3)}")
        #show_item_colorized(vertexEntry)
        #vNormEntry, vTangEntry = publish_vectors(vertexGeom, dataPoint)
        ##vNormEntry = geompy.addToStudyInFather(vertexGeom, vNorm, "vNorm")
        ##vTangEntry = geompy.addToStudyInFather(vertexGeom, vTang, "vTang")
        #show_item_colorized(vNormEntry)
        #show_item_colorized(vTangEntry)
        #TODO - Display point couleur + vecteurs et forcer le study show
        #raise Orthogonality_error(f"The angle between the vectors at point {vertexEntry} is {round(angle,3)}deg. Cos = {cosa}")
        return True
    return False


def vectors_angle(norm:cls_vector, tan:cls_vector):
    cosa = (norm.vx*tan.vx + norm.vy*tan.vy + norm.vz*tan.vz)/(np.sqrt(np.power(norm.vx, 2) + np.power(norm.vy, 2) + np.power(norm.vz, 2)) * np.sqrt(np.power(tan.vx, 2) + np.power(tan.vy, 2) + np.power(tan.vz, 2)))
    return np.rad2deg(np.arccos(cosa)), cosa


def delete_geom(geomList):
    #https://docs.salome-platform.org/latest/gui/GEOM/geompy_doc/classgeomtools_1_1GeomStudyTools.html#aa6a7aa2538b85405cfef24b8a4daaf5f
    for geom in geomList:
        geom.Destroy()
    return


#@reloading
def approach_points(curve:cls_curve, point:cls_point, secuOption:bool, clearOption:bool, securityGeom, vertexGeom, curveGeom, fabMode:int):
    secuPoint = None
    clearPoint = None
    if secuOption:
        secuPoint = security_point(point, curve, vertexGeom, curveGeom, securityGeom, fabMode)
        #curve.add_point_to_curve(secuPoint)
    if clearOption:
        clearPoint = clear_point(curve, point, APPROACH, fabMode, curveGeom, distOnApproach=0)
        #curve.add_point_to_curve(clearPoint)
        #if fabMode == tapeLayingAirPulse:
        #    intPtsLst = inter_points(curve, clearPoint, APPROACH, fabMode)
    return secuPoint, clearPoint


def retract_points(curve:cls_curve, point:cls_point, secuOption:bool, clearOption:bool, securityGeom, vertexGeom, curveGeom, fabMode:int):
    secuPoint = None
    clearPoint = None
    if clearOption:
        #if fabMode == tapeLayingAirPulse:
        #    intPtsLst = inter_points(curve, point, RETRACT, fabMode)
        clearPoint = clear_point(curve, point, RETRACT, fabMode, curveGeom)
        #curve.add_point_to_curve(clearPoint)
    if secuOption:
        secuPoint = security_point(clearPoint, curve, vertexGeom, curveGeom, securityGeom, fabMode)
        #curve.add_point_to_curve(secuPoint)
    return secuPoint, clearPoint


#@reloading
def clear_point(curve:cls_curve, point:cls_point, moveType:int, fabMode:int, curveGeom, distOnApproach:float=None):
    entry, coordinates = clear_offset(point, curveGeom, moveType_interpreter(moveType))
    speed = speed_selector(moveType, fabMode)
    ths = tool_head_state(fabMode, moveType, 0, curve.curveLength, point.distanceOnCurve, distOnApproach)
    clearPoint = cls_point(coordinates, entry, point.distanceOnCurve, moveType, None, 
                      point.normalVector, point.tangentialVector, ths, speed, True, 0)
    if moveType == RETRACT and fabMode == tapeLayingAirPulse:
        intPtsLst = inter_points(curve, point, moveType, fabMode)
    curve.add_point_to_curve(clearPoint)
    if moveType == APPROACH and fabMode == tapeLayingAirPulse:
            intPtsLst = inter_points(curve, clearPoint, moveType, fabMode)
    return clearPoint


def clear_offset(point:cls_point, curveGeom, descriptor:str):
    coord = offset_point(point, dataStruct.machineParam.generics.securityDistance)
    vertexGeom = geompy.MakeVertex(coord.x, coord.y, coord.z)
    entry, coordinates = add_to_study(curveGeom, vertexGeom, descriptor)
    return entry, coordinates


def offset_point(point:cls_point, distance:float):
    ofX = point.coordinates.x + point.normalVector.vx * distance
    ofY = point.coordinates.y + point.normalVector.vy * distance
    ofZ = point.coordinates.z + point.normalVector.vz * distance
    return cls_coordinates([ofX, ofY, ofZ])


def inter_points(curve:cls_curve, oriPoint:cls_point, moveType:int, fabMode:int):
    interPointList = []
    nbInterPts = round(dataStruct.machineParam.generics.securityDistance / dataStruct.machineParam.airTape.securityDiscretisation)
    if nbInterPts > 1:
        secuDiscr = dataStruct.machineParam.generics.securityDistance / nbInterPts
        if moveType == APPROACH : 
            secuDiscr = -secuDiscr  #Negative distance
        for i in range(1, nbInterPts): #first point already created
            distOnApproach = i*secuDiscr
            coordinates = offset_point(oriPoint, distOnApproach)
            speed = speed_selector(moveType, fabMode)
            if moveType == APPROACH:
                distOnApproach = -distOnApproach
            ths = tool_head_state(fabMode, moveType, secuDiscr, curve.curveLength, None, distOnApproach)
            #print(f"dstOAp {distOnApproach} THS ",ths)
            interPoint = cls_point(coordinates, "Unpublished", 0, moveType, None, 
                                   oriPoint.normalVector, oriPoint.tangentialVector, ths, speed, False, 0)
            curve.add_point_to_curve(interPoint)
            interPointList.append(interPoint)
    return interPointList


#@reloading
def security_point(existingPoint:cls_point, curve:cls_curve, vertexGeom, curveGeom, securityGeom, fabMode):
    if securityGeom == None:
        return None
    projectedGeom = geompy.MakeProjection(vertexGeom, securityGeom) #TODO - Must be planar or cylindrical face
    vertexEntry, coordinates = add_to_study(curveGeom, projectedGeom, moveType_interpreter(TRAVEL))
    ths = tool_head_state(fabMode, TRAVEL, 0, curve.curveLength, existingPoint.distanceOnCurve)
    speed = speed_selector(TRAVEL, fabMode)
    point = cls_point(coordinates, vertexEntry, existingPoint.distanceOnCurve, TRAVEL, securityGeom.GetEntry(), 
                      existingPoint.normalVector, existingPoint.tangentialVector, ths, speed, False, 0)  #salome.ObjectToID(securityGeom)
    curve.add_point_to_curve(point)
    return point
    

def publish_vectors(vertexGeom, dataPoint:cls_point):
    vNorm = geompy.MakeVector(vertexGeom, geompy.MakeVertex(dataPoint.coordinates.x + dataPoint.normalVector.vx, dataPoint.coordinates.y + dataPoint.normalVector.vy, dataPoint.coordinates.z + dataPoint.normalVector.vz)) #geompy.MakeVectorDXDYDZ(dataNormalVector.vx, dataNormalVector.vy, dataNormalVector.vz)
    vTang = geompy.MakeVector(vertexGeom, geompy.MakeVertex(dataPoint.coordinates.x + dataPoint.tangentialVector.vx, dataPoint.coordinates.y + dataPoint.tangentialVector.vy, dataPoint.coordinates.z + dataPoint.tangentialVector.vz)) #geompy.MakeVectorDXDYDZ(dataTangentialVector.vx, dataTangentialVector.vy, dataTangentialVector.vz)
    if not doNotPublishPointInStudy:
        vNormEntry = geompy.addToStudyInFather(vertexGeom, vNorm, "vNorm")
        vTangEntry = geompy.addToStudyInFather(vertexGeom, vTang, "vTang")
    else :
        vNormEntry = "Unpublished"
        vTangEntry = "Unpublished"
    return vNormEntry, vTangEntry
