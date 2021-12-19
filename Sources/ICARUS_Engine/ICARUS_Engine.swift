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
    
    init(posVec1 : vector , posVec2 : vector, posVec3 : vector,flipNorm : Bool = false,color : Color = .white,thickness : Float = 0.25){
            
            self.posVec1 = posVec1
            self.posVec2 = posVec2
            self.posVec3 = posVec3
            self.norm = vector(x: 0, y: 0, z: 0)
            self.color = color
            self.thickness = thickness
        if(flipNorm == false){
            self.norm = crossProd(vecA: self.posVec1, vecB: self.posVec2, vecC: self.posVec3)
        }else{
            self.norm = crossProd(vecA: self.posVec1, vecB: self.posVec2, vecC: self.posVec3)
            self.norm.x = -self.norm.x
            self.norm.y = -self.norm.y
            self.norm.z = -self.norm.z
        }
        
            
        }
    
    func crossProd(vecA : vector,vecB : vector,vecC : vector) -> vector{
        let A = vector(x: vecA.x-vecB.x, y: vecA.y-vecB.y, z: vecA.z-vecB.z)
        let B = vector(x: vecC.x-vecB.x, y: vecC.y-vecB.y, z: vecC.z-vecB.z)
        let vecNorm = vector(x: A.y*B.z - A.z*B.y , y: A.z*B.x - A.x*B.z, z: A.x*B.y - A.y*B.x)
        return vecNorm
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
            let A = vector(x: self.Xpos, y: self.Ypos, z: self.Zpos)
            let B = vector(x: self.Xpos + self.width, y: self.Ypos, z: self.Zpos)
            let C = vector(x: self.Xpos + self.width, y: self.Ypos, z: self.Zpos + self.depth)
            let D = vector(x: self.Xpos, y: self.Ypos, z: self.Zpos + self.depth)
            let E = vector(x: A.x, y: A.y + self.height, z: A.z)
            let F = vector(x: B.x, y: B.y + self.height, z: B.z)
            let G = vector(x: C.x, y: C.y + self.height, z: C.z)
            let H = vector(x: D.x, y: D.y + self.height, z: D.z)
            
            self.Faces.append(Tri(posVec1: A, posVec2: E, posVec3: F,flipNorm: true))
            self.Faces.append(Tri(posVec1: A, posVec2: B, posVec3: F,flipNorm: false))
            self.Faces.append(Tri(posVec1: B, posVec2: F, posVec3: G,flipNorm: true))
            self.Faces.append(Tri(posVec1: B, posVec2: C, posVec3: G,flipNorm: false))
            self.Faces.append(Tri(posVec1: C, posVec2: G, posVec3: H,flipNorm: true))
            self.Faces.append(Tri(posVec1: C, posVec2: D, posVec3: H,flipNorm: false))
            self.Faces.append(Tri(posVec1: D, posVec2: H, posVec3: E,flipNorm: true))
            self.Faces.append(Tri(posVec1: D, posVec2: A, posVec3: E,flipNorm: false))
            self.Faces.append(Tri(posVec1: B, posVec2: A, posVec3: D,flipNorm: false))
            self.Faces.append(Tri(posVec1: B, posVec2: C, posVec3: D,flipNorm: true))
            self.Faces.append(Tri(posVec1: F, posVec2: E, posVec3: H,flipNorm: true))
            self.Faces.append(Tri(posVec1: F, posVec2: G, posVec3: H,flipNorm: false))
        }
    
        
    }

    
@available(macOS 10.15, *)
class Xaxis : OBJS_3D{
    var axis = Tri(posVec1: vector(x: 0, y: 0, z: 0), posVec2: vector(x: 1000, y: 0, z: 0), posVec3: vector(x: -1000, y: 0, z: 0))
    }
@available(macOS 10.15, *)
class Yaxis : OBJS_3D{
    var axis = Tri(posVec1: vector(x: 0, y: 0, z: 0), posVec2: vector(x: 0, y: 1000, z: 0), posVec3: vector(x: 0, y: -1000, z: 0))
    }
@available(macOS 10.15, *)
class Zaxis : OBJS_3D{
    var axis = Tri(posVec1: vector(x: 0, y: 0, z: 0), posVec2: vector(x: 0, y: 0, z: 1000), posVec3: vector(x: 0, y: 0, z: -1000))
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
        
        return path.stroke(.black, lineWidth: 0.25) as! Path
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
                    let a = Zres[i]
                    if(a > 0){
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
                    tri[j].pointee.posVec3.point.x = Xres[i]
                    tri[j].pointee.posVec3.point.y = Yres[i]
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




