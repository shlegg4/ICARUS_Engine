import SwiftUI
import matrixLib


public enum Projection{
    
    case ortho
    case perspective
    
}


//    Triangle class is used to define faces in all other 3d objects
@available(macOS 10.15, *)
public struct Tri{
        var norm : vector
        var isVis : Bool = true
        var posVec1 : vector
        var posVec2 : vector
        var posVec3 : vector
        var color : Color
        var thickness : Float
    @available(macOS 10.15, *)
    
        init(normX : Float, normY : Float, normZ : Float, posVec1X : Float, posVec1Y : Float, posVec1Z : Float, posVec2X : Float, posVec2Y : Float, posVec2Z : Float, posVec3X : Float, posVec3Y : Float, posVec3Z : Float,color : Color = .white,thickness : Float = 0.25){
            self.norm = vector(x: normX,y: normY,z: normZ)
            self.posVec1 = vector(x: posVec1X,y: posVec1Y,z: posVec1Z)
            self.posVec2 = vector(x: posVec2X,y: posVec2Y,z: posVec2Z)
            self.posVec3 = vector(x: posVec3X,y: posVec3Y,z: posVec3Z)
            self.color = color
            self.thickness = thickness
        }
    }
@available(macOS 10.15, *)
extension Tri : Equatable{
    public static func == (lhs: Tri, rhs: Tri) -> Bool {
        return lhs.posVec1.x == rhs.posVec1.x && lhs.posVec2.x == rhs.posVec2.x && lhs.posVec3.x == rhs.posVec3.x && lhs.posVec1.y == rhs.posVec1.y && lhs.posVec2.y == rhs.posVec2.y && lhs.posVec3.y == rhs.posVec3.y && lhs.posVec1.z == rhs.posVec1.z && lhs.posVec2.z == rhs.posVec2.z && lhs.posVec3.z == rhs.posVec3.z
    }
    
    
    
}

    /// Create a box
@available(macOS 10.15, *)
public class Box : OBJS_3D{
        
    public var height : Float
    public var depth : Float
    public var width : Float
        
        /// Box initialiser
        /// - Parameters:
        ///   - x: X position of vertex defining box
        ///   - y: Y position of vertex defining box
        ///   - z: Z position of vertex defining box
        ///   - width: Width of the box
        ///   - height: Height of the box
        ///   - depth: Depth of the box
        public init(x : Float, y : Float, z : Float, width : Float, height : Float, depth : Float){
            
            self.depth = depth // Z axis
            self.height = height // Y axis
            self.width = width // X axis
            
            super.init(Xpos: x, Ypos: y, Zpos: z)
            
            
//            Create all faces associated with the sides of a box
            let ax = self.Xpos
            let ay = self.Ypos
            let az = self.Zpos
            
            let bx = self.Xpos + self.width
            let by = self.Ypos
            let bz = self.Zpos
            
            let cx = self.Xpos + self.width
            let cy = self.Ypos
            let cz = self.Zpos + self.depth
            
            let dx = self.Xpos
            let dy = self.Ypos
            let dz = self.Zpos + self.depth
            
            let ex = ax
            let ey = ay + self.height
            let ez = az
            
            let fx = bx
            let fy = by + self.height
            let fz = bz
            
            let gx = cx
            let gy = cy + self.height
            let gz = cz
            
            let hx = dx
            let hy = dy + self.height
            let hz = dz
            
            let AEFnorm = crossProd(ax: ax, ay: ay, az: az, bx: ex, by: ey, bz: ez, cx: fx, cy: fy, cz: fz)
            let ABFnorm = crossProd(ax: ax, ay: ay, az: az, bx: bx, by: by, bz: bz, cx: fx, cy: fy, cz: fz)
            let BFGnorm = crossProd(ax: bx, ay: by, az: bz, bx: fx, by: fy, bz: fz, cx: gx, cy: gy, cz: gz)
            let BCGnorm = crossProd(ax: bx, ay: by, az: bz, bx: cx, by: cy, bz: cz, cx: gx, cy: gy, cz: gz)
            let CGHnorm = crossProd(ax: cx, ay: cy, az: cz, bx: gx, by: gy, bz: gz, cx: hx, cy: hy, cz: hz)
            let CDHnorm = crossProd(ax: cx, ay: cy, az: cz, bx: dx, by: dy, bz: dz, cx: hx, cy: hy, cz: hz)
            let DHEnorm = crossProd(ax: dx, ay: dy, az: dz, bx: hx, by: hy, bz: hz, cx: ex, cy: ey, cz: ez)
            let DAEnorm = crossProd(ax: dx, ay: dy, az: dz, bx: ax, by: ay, bz: az, cx: ex, cy: ey, cz: ez)
            let ABDnorm = crossProd(ax: ax, ay: ay, az: az, bx: bx, by: by, bz: bz, cx: dx, cy: dy, cz: dz)
            let BCDnorm = crossProd(ax: bx, ay: by, az: bz, bx: cx, by: cy, bz: cz, cx: dx, cy: dy, cz: dz)
            let EFHnorm = crossProd(ax: ex, ay: ey, az: ez, bx: fx, by: fy, bz: fz, cx: hx, cy: hy, cz: hz)
            let FGHnorm = crossProd(ax: fx, ay: fy, az: fz, bx: gx, by: gy, bz: gz, cx: hx, cy: hy, cz: hz)
            
            self.Faces.append(Tri(normX: AEFnorm.x, normY: AEFnorm.y, normZ: AEFnorm.z, posVec1X: ax, posVec1Y: ay, posVec1Z: az, posVec2X: ex, posVec2Y: ey, posVec2Z: ez, posVec3X: fx, posVec3Y: fy, posVec3Z: fz))
            self.Faces.append(Tri(normX: ABFnorm.x, normY: ABFnorm.y, normZ: ABFnorm.z, posVec1X: ax, posVec1Y: ay, posVec1Z: az, posVec2X: bx, posVec2Y: by, posVec2Z: bz, posVec3X: fx, posVec3Y: fy, posVec3Z: fz))
            self.Faces.append(Tri(normX: BFGnorm.x, normY: BFGnorm.y, normZ: BFGnorm.z, posVec1X: bx, posVec1Y: by, posVec1Z: bz, posVec2X: fx, posVec2Y: fy, posVec2Z: fz, posVec3X: gx, posVec3Y: gy, posVec3Z: gz))
            self.Faces.append(Tri(normX: BCGnorm.x, normY: BCGnorm.y, normZ: BCGnorm.z, posVec1X: bx, posVec1Y: by, posVec1Z: bz, posVec2X: cx, posVec2Y: cy, posVec2Z: cz, posVec3X: gx, posVec3Y: gy, posVec3Z: gz))
            self.Faces.append(Tri(normX: CGHnorm.x, normY: CGHnorm.y, normZ: CGHnorm.z, posVec1X: cx, posVec1Y: cy, posVec1Z: cz, posVec2X: gx, posVec2Y: gy, posVec2Z: gz, posVec3X: hx, posVec3Y: hy, posVec3Z: hz))
            self.Faces.append(Tri(normX: CDHnorm.x, normY: CDHnorm.y, normZ: CDHnorm.z, posVec1X: cx, posVec1Y: cy, posVec1Z: cz, posVec2X: dx, posVec2Y: dy, posVec2Z: dz, posVec3X: hx, posVec3Y: hy, posVec3Z: hz))
            self.Faces.append(Tri(normX: DHEnorm.x, normY: DHEnorm.y, normZ: DHEnorm.z, posVec1X: dx, posVec1Y: dy, posVec1Z: dz, posVec2X: hx, posVec2Y: hy, posVec2Z: hz, posVec3X: ex, posVec3Y: ey, posVec3Z: ez))
            self.Faces.append(Tri(normX: DAEnorm.x, normY: DAEnorm.y, normZ: DAEnorm.z, posVec1X: dx, posVec1Y: dy, posVec1Z: dz, posVec2X: ax, posVec2Y: ay, posVec2Z: az, posVec3X: ex, posVec3Y: ey, posVec3Z: ez))
            self.Faces.append(Tri(normX: ABDnorm.x, normY: ABDnorm.y, normZ: ABDnorm.z, posVec1X: ax, posVec1Y: ay, posVec1Z: az, posVec2X: bx, posVec2Y: by, posVec2Z: bz, posVec3X: dx, posVec3Y: dy, posVec3Z: dz))
            self.Faces.append(Tri(normX: BCDnorm.x, normY: BCDnorm.y, normZ: BCDnorm.z, posVec1X: bx, posVec1Y: by, posVec1Z: bz, posVec2X: cx, posVec2Y: cy, posVec2Z: cz, posVec3X: dx, posVec3Y: dy, posVec3Z: dz))
            self.Faces.append(Tri(normX: EFHnorm.x, normY: EFHnorm.y, normZ: EFHnorm.z, posVec1X: ex, posVec1Y: ey, posVec1Z: ez, posVec2X: fx, posVec2Y: fy, posVec2Z: fz, posVec3X: hx, posVec3Y: hy, posVec3Z: hz))
            self.Faces.append(Tri(normX: FGHnorm.x, normY: FGHnorm.y, normZ: FGHnorm.z, posVec1X: fx, posVec1Y: fy, posVec1Z: fz, posVec2X: gx, posVec2Y: gy, posVec2Z: gz, posVec3X: hx, posVec3Y: hy, posVec3Z: hz))
        }
    func crossProd(ax:Float,ay:Float,az:Float,bx:Float,by:Float,bz:Float,cx:Float,cy:Float,cz:Float) -> (x : Float,y : Float,z : Float){
        let vecABx = bx - ax
        let vecABy = by - ay
        let vecABz = bz - az
        let vecCBx = bx - cx
        let vecCBy = by - cy
        let vecCBz = bz - cz
        
        let normX = vecABy*vecCBz - vecABz*vecCBy
        let normY = vecABz*vecCBx - vecABx*vecCBz
        let normZ = vecABx*vecCBy - vecABy*vecCBx
        return (normX,normY,normZ)
    }
        
    }

    
@available(macOS 10.15, *)
class Xaxis : OBJS_3D{
        var axis = Tri(normX: 0, normY: 0, normZ: 0, posVec1X: 0, posVec1Y: 0, posVec1Z: 0, posVec2X: 1000, posVec2Y: 0, posVec2Z: 0, posVec3X: -1000, posVec3Y: 0, posVec3Z: 0,color: .red,thickness: 0.75)
    }
@available(macOS 10.15, *)
class Yaxis : OBJS_3D{
        var axis = Tri(normX: 0, normY: 0, normZ: 0, posVec1X: 0, posVec1Y: 0, posVec1Z: 0, posVec2X: 0, posVec2Y: 1000, posVec2Z: 0, posVec3X: 0, posVec3Y: -1000, posVec3Z: 0,color: .yellow,thickness: 0.75)
    }
@available(macOS 10.15, *)
class Zaxis : OBJS_3D{
        var axis = Tri(normX: 0, normY: 0, normZ: 0, posVec1X: 0, posVec1Y: 0, posVec1Z: 0, posVec2X: 0, posVec2Y: 0, posVec2Z: 1000, posVec3X: 0, posVec3Y: 0, posVec3Z: -1000,color: .blue,thickness: 0.75)
    }
    


@available(macOS 10.15, *)
public class OBJS_3D : Identifiable {
    public let id = UUID()
    
    public var Xpos : Float
    public var Ypos : Float
    public var Zpos : Float
    
    public var isVisible : Bool
    public var Colour : Color
    
    public var Faces : [Tri] = []
    
    public init( Xpos : Float, Ypos : Float, Zpos : Float,Colour : Color = .white, isVisible : Bool = true ){
        
        self.Xpos = Xpos
        self.Ypos = Ypos
        self.Zpos = Zpos
        
        self.isVisible = isVisible
        self.Colour = Colour
        
    }
    
    public func Draw() -> Path{
        var path : Path = Path()
        for tri in Faces{
            if(tri.isVis == true){
                var temp = Path()
                temp.move(to: CGPoint(x: Int(tri.posVec1.point.x), y: Int(tri.posVec1.point.y)))
                temp.addLine(to: CGPoint(x: Int(tri.posVec2.point.x), y: Int(tri.posVec2.point.y)))
                temp.addLine(to: CGPoint(x: Int(tri.posVec3.point.x), y: Int(tri.posVec3.point.y)))
                temp.addLine(to: CGPoint(x: Int(tri.posVec1.point.x), y: Int(tri.posVec1.point.y)))
                path.addPath(temp)
            }
        }
        
        return path
    }
    
//    Possible functions Append to this list : Colour Faces,
    
    
}

@available(macOS 10.15, *)
extension OBJS_3D : Equatable{
    public static func == (lhs: OBJS_3D, rhs: OBJS_3D) -> Bool {
        return lhs.Faces == rhs.Faces
    }
    
    
    
}
@available(macOS 10.15, *)
public struct ICARUS {
    
    public var pitch : Float = 0
    public var roll : Float = 0
    public var yaw : Float = 0
    public var Zoom : Float = 1
    
    public var xTranslate : Float = 0
    public var yTranslate : Float = 0
    public var zTranslate : Float = 0
    
    
    public var projection : Projection
    public var objList : [OBJS_3D] = []
    
//    Initialise the projection method
    public init (projection : Projection){
        self.projection = projection
    }
    
//    Call this function in order to update the view
    public func Update(){
        
        let projectionMatrix = ortho(pitch: self.pitch, roll: self.roll, yaw: self.yaw, scale: self.Zoom)
        var xPoints : [Float] = []
        var yPoints : [Float] = []
        var zPoints : [Float] = []
        var tris : [UnsafeMutablePointer<Tri>] = []
        
        for i in 0 ..< self.objList.endIndex{
            for j in 0 ..< self.objList[i].Faces.endIndex{
                tris.append(&self.objList[i].Faces[j])
                let tri = self.objList[i].Faces[j]
                xPoints.append(tri.norm.x)
                xPoints.append(tri.posVec1.x)
                xPoints.append(tri.posVec2.x) // Append x,y,z position of obj face to corresponding list
                xPoints.append(tri.posVec3.x)
                
                yPoints.append(tri.norm.y)
                yPoints.append(tri.posVec1.y)
                yPoints.append(tri.posVec2.y)
                yPoints.append(tri.posVec3.y)
                
                zPoints.append(tri.norm.z)
                zPoints.append(tri.posVec1.z)
                zPoints.append(tri.posVec2.z)
                zPoints.append(tri.posVec3.z)
            }
        }
        
        let device = MTLCreateSystemDefaultDevice()
        let projector = MetalProjector(device: device!)
        projector.FillBuffers(arrX: xPoints, arrY: yPoints, arrZ: zPoints, scaX1: projectionMatrix[0,0], scaY1: projectionMatrix[0,1], scaZ1: projectionMatrix[0,2], scaX2: projectionMatrix[1,0], scaY2: projectionMatrix[1,1], scaZ2: projectionMatrix[1,2],scaX3: projectionMatrix[2,0],scaY3: projectionMatrix[2,1],scaZ3: projectionMatrix[2,2], Xtranslate: self.xTranslate, Ytranslate:  self.yTranslate, Ztranslate: self.zTranslate)
        projector.SendComputeCommand()
        projector.ReturnXYList(tri: tris)
        
    }
    
//    Add object to list of objects to display
    public mutating func Create(obj : OBJS_3D){
        self.objList.append(obj)
    }
    
//    Remove object from list of objects to display
    public mutating func Delete(obj : OBJS_3D){
        if let index = self.objList.firstIndex(of: obj){
            self.objList.remove(at: index)
        }
    }
        
        /// orthographic projection
        /// - Parameters:
        ///   - pitch: rotation about the y axis
        ///   - roll: rotation about the z axis
        ///   - yaw: rotation about the x axis
        ///   - scale: enlargement of the image
    func ortho(pitch:Float,roll:Float,yaw:Float,scale:Float) -> Matrix{
        let pitchMat = Matrix(rows: 3, columns: 3, type: .custMatrix, data: [cos(pitch),0,sin(pitch),0,1,0,-sin(pitch),0,cos(pitch)])
        let rollMat = Matrix(rows: 3, columns: 3, type: .custMatrix, data: [1,0,0,0,cos(roll),-sin(roll),0,sin(roll),cos(roll)])
        let yawMat = Matrix(rows: 3, columns: 3, type: .custMatrix, data: [cos(yaw),-sin(yaw),0,sin(yaw),cos(yaw),0,0,0,1])
        
        let transform = Matrix.scalar_multiply(matA: Matrix.multiply(matA: rollMat,matB: Matrix.multiply(matA: pitchMat, matB: yawMat)),scalar: scale)
        return transform
    }
        
        
    class MetalProjector{
        var error : NSError?
        var device : MTLDevice!
        var addFunctionPSO : MTLComputePipelineState!
        var scalarMultiplyFunctionPSO : MTLComputePipelineState!
        var addScalarFunctionPSO : MTLComputePipelineState!
        var commandQueue : MTLCommandQueue!
        
        
        
        
        
    //    vector input buffers
        var bufferx : MTLBuffer!
        var buffery : MTLBuffer!
        var bufferz : MTLBuffer!
        
    // temporary buffers for holding result of scalar multiplication
        var bufferXres : MTLBuffer!
        var bufferYres : MTLBuffer!
        var bufferZres : MTLBuffer!
        
        var Xtranslate : Float = 0
        var Ytranslate : Float = 0
        var Ztranslate : Float  = 0
        
        var scaX1 : Float = 0
        var scaY1 : Float = 0
        var scaZ1 : Float = 0
        var scaX2 : Float = 0
        var scaY2 : Float = 0
        var scaZ2 : Float = 0
        var scaX3 : Float = 0
        var scaY3 : Float = 0
        var scaZ3 : Float = 0
        

        
    //      vector output buffers
        var bufferXRes : MTLBuffer!
        var bufferYRes : MTLBuffer!
        var bufferZRes : MTLBuffer!
        
        var arrayLength : Int = 0
        
        
        
        init (device : MTLDevice){
            self.device = device
            let defaultLibrary : MTLLibrary! = try? self.device.makeDefaultLibrary(bundle: Bundle.module)
            let addFunction : MTLFunction! = defaultLibrary.makeFunction(name: "add_arrays")
            let scalarMultiplyFunction : MTLFunction! = defaultLibrary.makeFunction(name: "multiply_array_by_scalar")
            let addScalarFunction : MTLFunction! = defaultLibrary.makeFunction(name: "addArray_by_scalar")
            self.addScalarFunctionPSO = try! self.device.makeComputePipelineState(function: addScalarFunction)
            self.addFunctionPSO = try! self.device.makeComputePipelineState(function: addFunction)
            self.scalarMultiplyFunctionPSO = try! self.device.makeComputePipelineState(function: scalarMultiplyFunction)
            self.commandQueue = self.device.makeCommandQueue()
            
        }
        func FillBuffers(arrX : [Float], arrY : [Float], arrZ : [Float], scaX1 : Float , scaY1 : Float , scaZ1 : Float,scaX2 : Float , scaY2 : Float , scaZ2 : Float,scaX3 : Float,scaY3 :Float, scaZ3 : Float,Xtranslate : Float,Ytranslate : Float, Ztranslate : Float){
            self.arrayLength = arrX.count
            let BufferSize = arrX.count * MemoryLayout<Float>.size
            
                self.bufferx = self.device.makeBuffer(length: BufferSize, options: MTLResourceOptions.storageModeShared)
                self.buffery = self.device.makeBuffer(length: BufferSize, options: MTLResourceOptions.storageModeShared)
                self.bufferz = self.device.makeBuffer(length: BufferSize, options: MTLResourceOptions.storageModeShared)
            
            self.bufferXres = self.device.makeBuffer(length: BufferSize, options: MTLResourceOptions.storageModeShared)
            self.bufferYres = self.device.makeBuffer(length: BufferSize, options: MTLResourceOptions.storageModeShared)
            self.bufferZres = self.device.makeBuffer(length: BufferSize, options: MTLResourceOptions.storageModeShared)
            self.bufferXRes = self.device.makeBuffer(length: BufferSize, options: MTLResourceOptions.storageModeShared)
            self.bufferYRes = self.device.makeBuffer(length: BufferSize, options: MTLResourceOptions.storageModeShared)
            self.bufferZRes = self.device.makeBuffer(length: BufferSize, options: MTLResourceOptions.storageModeShared)
            
            self.scaX1 = scaX1
            self.scaY1 = scaY1
            self.scaZ1 = scaZ1
            self.scaX2 = scaX2
            self.scaY2 = scaY2
            self.scaZ2 = scaZ2
            self.scaX3 = scaX3
            self.scaY3 = scaY3
            self.scaZ3 = scaZ3
            
            self.Xtranslate = Xtranslate
            self.Ytranslate = Ytranslate
            self.Ztranslate = Ztranslate

            
            
            self.loadData(buffer: self.bufferx, arr: arrX)
            self.loadData(buffer: self.buffery, arr: arrY)
            self.loadData(buffer: self.bufferz, arr: arrZ)
        }
        func loadData(buffer : MTLBuffer , sca : Float){
            let dataPTR = buffer.contents().bindMemory(to: Float.self, capacity: 1)
            dataPTR[0] = sca
        }
        
        func loadData(buffer : MTLBuffer,arr : [Float]){
            let dataPtr = buffer.contents().bindMemory(to: Float.self, capacity: arr.count)
            for i in 0 ..< arr.count {
                dataPtr[i] = arr[i]
            }
        }
        
        func encodeScalarCommands(computeEncoder : MTLComputeCommandEncoder,buffer : MTLBuffer, Sca : inout Float, bufferRes : MTLBuffer?){
            computeEncoder.setComputePipelineState(self.scalarMultiplyFunctionPSO)
            computeEncoder.setBuffer(buffer, offset: 0, index: 0)
            computeEncoder.setBytes(&Sca, length: MemoryLayout<Float>.size, index: 1)
            computeEncoder.setBuffer(bufferRes, offset: 0, index: 2)
            let gridSize : MTLSize = MTLSizeMake(self.arrayLength, 1, 1)
            
            var threadGroupSizeInt : Int = self.scalarMultiplyFunctionPSO.maxTotalThreadsPerThreadgroup
            
            if(threadGroupSizeInt > self.arrayLength){
                threadGroupSizeInt = self.arrayLength
            }
            
            let threadGroupSize : MTLSize = MTLSizeMake(threadGroupSizeInt, 1, 1)
            
            computeEncoder.dispatchThreads(gridSize, threadsPerThreadgroup: threadGroupSize)
        }
        
        func encodeAddcommands(computeEncoder : MTLComputeCommandEncoder,bufferA : MTLBuffer, bufferB : MTLBuffer, bufferC : MTLBuffer,bufferRes : MTLBuffer){
            computeEncoder.setComputePipelineState(self.addFunctionPSO)
            computeEncoder.setBuffer(bufferA, offset: 0, index: 0)
            computeEncoder.setBuffer(bufferB, offset: 0, index: 1)
            computeEncoder.setBuffer(bufferC, offset: 0, index: 2)
            computeEncoder.setBuffer(bufferRes, offset: 0, index: 3)
            
            let gridSize : MTLSize = MTLSizeMake(arrayLength, 1, 1)
           
                   var threadGroupSizeInt : Int = self.addFunctionPSO.maxTotalThreadsPerThreadgroup
           
                   if(threadGroupSizeInt > arrayLength){
                       threadGroupSizeInt = arrayLength
                   }
           
                   let threadGroupSize : MTLSize = MTLSizeMake(threadGroupSizeInt, 1, 1)
           
                   computeEncoder.dispatchThreads(gridSize, threadsPerThreadgroup: threadGroupSize)
        }
        
        func encodeAddscalarCommands(computeEncoder : MTLComputeCommandEncoder,bufferA : MTLBuffer, Sca : inout Float, bufferRes : MTLBuffer?){
            computeEncoder.setComputePipelineState(self.addScalarFunctionPSO)
            computeEncoder.setBuffer(bufferA,offset: 0,index: 0)
            computeEncoder.setBytes(&Sca, length: MemoryLayout<Float>.size, index: 1)
            computeEncoder.setBuffer(bufferRes, offset: 0, index: 2)
            
            let gridSize : MTLSize = MTLSizeMake(arrayLength, 1, 1)
            
            var threadGroupSizeInt : Int = self.addScalarFunctionPSO.maxTotalThreadsPerThreadgroup
            
            if(threadGroupSizeInt > arrayLength){
                threadGroupSizeInt = arrayLength
            }
            
            let threadGroupSize : MTLSize = MTLSizeMake(threadGroupSizeInt, 1, 1)
            
            computeEncoder.dispatchThreads(gridSize, threadsPerThreadgroup: threadGroupSize)
            
        }
        
        
        func SendComputeCommand(){
            
            let commandBuffer: MTLCommandBuffer? = commandQueue.makeCommandBuffer()
            assert(commandBuffer != nil, "Command buffer = nil")
            
            let computeEncoder : MTLComputeCommandEncoder? = commandBuffer?.makeComputeCommandEncoder()
            assert(computeEncoder != nil, " Compute encoder = nil")
            
            
            self.encodeScalarCommands(computeEncoder: computeEncoder!,buffer: self.bufferx,Sca: &self.scaX1,bufferRes: self.bufferXres)
            self.encodeScalarCommands(computeEncoder: computeEncoder!,buffer: self.buffery,Sca: &self.scaY1,bufferRes: self.bufferYres)
            self.encodeScalarCommands(computeEncoder: computeEncoder!,buffer: self.bufferz,Sca: &self.scaZ1,bufferRes: self.bufferZres)
            self.encodeAddcommands(computeEncoder: computeEncoder!, bufferA: self.bufferXres, bufferB: self.bufferYres, bufferC: self.bufferZres, bufferRes: self.bufferXRes)
            self.encodeScalarCommands(computeEncoder: computeEncoder!,buffer: self.bufferx,Sca: &self.scaX2,bufferRes: self.bufferXres)
            self.encodeScalarCommands(computeEncoder: computeEncoder!,buffer: self.buffery,Sca: &self.scaY2,bufferRes: self.bufferYres)
            self.encodeScalarCommands(computeEncoder: computeEncoder!,buffer: self.bufferz,Sca: &self.scaZ2,bufferRes: self.bufferZres)
            self.encodeAddcommands(computeEncoder: computeEncoder!, bufferA: self.bufferXres, bufferB: self.bufferYres, bufferC: self.bufferZres, bufferRes: self.bufferYRes)
            self.encodeScalarCommands(computeEncoder: computeEncoder!,buffer: self.bufferx,Sca: &self.scaX3,bufferRes: self.bufferXres)
            self.encodeScalarCommands(computeEncoder: computeEncoder!,buffer: self.buffery,Sca: &self.scaY3,bufferRes: self.bufferYres)
            self.encodeScalarCommands(computeEncoder: computeEncoder!,buffer: self.bufferz,Sca: &self.scaZ3,bufferRes: self.bufferZres)
            self.encodeAddcommands(computeEncoder: computeEncoder!, bufferA: self.bufferXres, bufferB: self.bufferYres, bufferC: self.bufferZres, bufferRes: self.bufferZRes)
            
            self.encodeAddscalarCommands(computeEncoder: computeEncoder!, bufferA: self.bufferXRes, Sca: &self.Xtranslate, bufferRes: self.bufferXRes)
            self.encodeAddscalarCommands(computeEncoder: computeEncoder!, bufferA: self.bufferYRes, Sca: &self.Ytranslate, bufferRes: self.bufferYRes)
            
            computeEncoder?.endEncoding()
            commandBuffer?.commit()
            commandBuffer?.waitUntilCompleted()
            
            
        }
        
        func ReturnXYList(tri : [UnsafeMutablePointer<Tri>]){
            let Xres = self.bufferXRes.contents().bindMemory(to: Float.self, capacity: self.bufferx.allocatedSize/MemoryLayout<Float>.size)
            let Yres = self.bufferYRes.contents().bindMemory(to: Float.self, capacity: self.bufferx.allocatedSize/MemoryLayout<Float>.size)
            let Zres = self.bufferZRes.contents().bindMemory(to: Float.self, capacity: self.bufferx.allocatedSize/MemoryLayout<Float>.size)
            
            var j = 0
            for i in 0..<self.arrayLength{
                
                if ((i) % 4 == 0){
                    tri[j].pointee.norm.point.x = Xres[i]
                    tri[j].pointee.norm.point.y = Yres[i]
                    if(Zres[i] <= 0){
                        tri[j].pointee.isVis = false
                    }else{
                        tri[j].pointee.isVis = true
                    }
                }
                if ((i-1) % 4 == 0){
                    tri[j].pointee.posVec1.point.x = Xres[i]
                    tri[j].pointee.posVec1.point.y = Yres[i]
                }
                if ((i-2) % 4 == 0){
                    tri[j].pointee.posVec2.point.x = Xres[i]
                    tri[j].pointee.posVec2.point.y = Yres[i]
                    
                }
                if ((i-3) % 4 == 0){
                    tri[j].pointee.posVec2.point.x = Xres[i]
                    tri[j].pointee.posVec2.point.y = Yres[i]
                    j+=1
                }
                
            }
            
        }
    }
}


public struct Point{
    var x : Float = 0
    var y : Float = 0
    init(x : Float,y : Float){
        self.x = x
        self.y = y
    }
}

public struct vector{
    var x : Float = 0
    var y : Float = 0
    var z : Float = 0
    var point : Point = Point(x: 0, y: 0)
    
    init(x:Float,y:Float,z:Float){
        self.x = x
        self.y = y
        self.z = z
    }
}




