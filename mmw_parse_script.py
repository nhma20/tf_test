# ****************************************************************************
# * (C) Copyright 2020, Texas Instruments Incorporated. - www.ti.com
# ****************************************************************************
# *
# *  Redistribution and use in source and binary forms, with or without
# *  modification, are permitted provided that the following conditions are
# *  met:
# *
# *    Redistributions of source code must retain the above copyright notice,
# *    this list of conditions and the following disclaimer.
# *
# *    Redistributions in binary form must reproduce the above copyright
# *    notice, this list of conditions and the following disclaimer in the
# *     documentation and/or other materials provided with the distribution.
# *
# *    Neither the name of Texas Instruments Incorporated nor the names of its
# *    contributors may be used to endorse or promote products derived from
# *    this software without specific prior written permission.
# *
# *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
# *  PARTICULAR TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# *  A PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  OWNER OR
# *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# *  EXEMPLARY, ORCONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# *  LIABILITY, WHETHER IN CONTRACT,  STRICT LIABILITY, OR TORT (INCLUDING
# *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# *
# ****************************************************************************

import serial
import time
import numpy as np
# import the parser function 
# from parser_mmw_demo import parser_one_mmw_demo_output_packet
# import the required Python packages
import struct
import math
import binascii
import codecs

# definations for parser pass/fail
TC_PASS   =  0
TC_FAIL   =  1

# Change the configuration file name
configFileName = '90deg_noGroup_18m_30Hz.cfg'

# Change the debug variable to use print()
DEBUG = False

# Constants
maxBufferSize = 2**15
CLIport = {}
Dataport = {}
byteBuffer = np.zeros(2**15,dtype = 'uint8')
byteBufferLength = 0
maxBufferSize = 2**15
magicWord = [2, 1, 4, 3, 6, 5, 8, 7]
detObj = {}  
frameData = {}    
currentIndex = 0
# word array to convert 4 bytes to a 32 bit number
word = [1, 2**8, 2**16, 2**24]




def getUint32(data):
    """!
       This function coverts 4 bytes to a 32-bit unsigned integer.

        @param data : 1-demension byte array  
        @return     : 32-bit unsigned integer
    """ 
    return (data[0] +
            data[1]*256 +
            data[2]*65536 +
            data[3]*16777216)

def getUint16(data):
    """!
       This function coverts 2 bytes to a 16-bit unsigned integer.

        @param data : 1-demension byte array
        @return     : 16-bit unsigned integer
    """ 
    return (data[0] +
            data[1]*256)

def getHex(data):
    """!
       This function coverts 4 bytes to a 32-bit unsigned integer in hex.

        @param data : 1-demension byte array
        @return     : 32-bit unsigned integer in hex
    """         
    #return (binascii.hexlify(data[::-1]))
    word = [1, 2**8, 2**16, 2**24]
    return np.matmul(data,word)

def checkMagicPattern(data):
    """!
       This function check if data arrary contains the magic pattern which is the start of one mmw demo output packet.  

        @param data : 1-demension byte array
        @return     : 1 if magic pattern is found
                      0 if magic pattern is not found 
    """ 
    found = 0
    if (data[0] == 2 and data[1] == 1 and data[2] == 4 and data[3] == 3 and data[4] == 6 and data[5] == 5 and data[6] == 8 and data[7] == 7):
        found = 1
    return (found)
          
def parser_helper(data, readNumBytes,debug=False):
    """!
       This function is called by parser_one_mmw_demo_output_packet() function or application to read the input buffer, find the magic number, header location, the length of frame, the number of detected object and the number of TLV contained in this mmw demo output packet.

        @param data                   : 1-demension byte array holds the the data read from mmw demo output. It ignorant of the fact that data is coming from UART directly or file read.  
        @param readNumBytes           : the number of bytes contained in this input byte array  
            
        @return headerStartIndex      : the mmw demo output packet header start location
        @return totalPacketNumBytes   : the mmw demo output packet lenght           
        @return numDetObj             : the number of detected objects contained in this mmw demo output packet          
        @return numTlv                : the number of TLV contained in this mmw demo output packet           
        @return subFrameNumber        : the sbuframe index (0,1,2 or 3) of the frame contained in this mmw demo output packet
    """ 
    
    headerStartIndex = -1

    for index in range (readNumBytes):
        if checkMagicPattern(data[index:index+8:1]) == 1:
            headerStartIndex = index
            break
  
    if headerStartIndex == -1: # does not find the magic number i.e output packet header 
        totalPacketNumBytes = -1
        numDetObj           = -1
        numTlv              = -1
        subFrameNumber      = -1
        platform            = -1
        frameNumber         = -1
        timeCpuCycles       = -1
    else: # find the magic number i.e output packet header 
        totalPacketNumBytes = getUint32(data[headerStartIndex+12:headerStartIndex+16:1])
        platform            = getHex(data[headerStartIndex+16:headerStartIndex+20:1])
        frameNumber         = getUint32(data[headerStartIndex+20:headerStartIndex+24:1])
        timeCpuCycles       = getUint32(data[headerStartIndex+24:headerStartIndex+28:1])
        numDetObj           = getUint32(data[headerStartIndex+28:headerStartIndex+32:1])
        numTlv              = getUint32(data[headerStartIndex+32:headerStartIndex+36:1])
        subFrameNumber      = getUint32(data[headerStartIndex+36:headerStartIndex+40:1])
        
    if(debug):
        print("headerStartIndex    = %d" % (headerStartIndex))
        print("totalPacketNumBytes = %d" % (totalPacketNumBytes))
        print("platform            = %s" % (platform)) 
        print("frameNumber         = %d" % (frameNumber)) 
        print("timeCpuCycles       = %d" % (timeCpuCycles))   
        print("numDetObj           = %d" % (numDetObj)) 
        print("numTlv              = %d" % (numTlv))
        print("subFrameNumber      = %d" % (subFrameNumber))   
                            
    return (headerStartIndex, totalPacketNumBytes, numDetObj, numTlv, subFrameNumber)


def parser_one_mmw_demo_output_packet(data, readNumBytes,debug=False):
    """!
       This function is called by application. Firstly it calls parser_helper() function to find the start location of the mmw demo output packet, then extract the contents from the output packet.
       Each invocation of this function handles only one frame at a time and user needs to manage looping around to parse data for multiple frames.

        @param data                   : 1-demension byte array holds the the data read from mmw demo output. It ignorant of the fact that data is coming from UART directly or file read.  
        @param readNumBytes           : the number of bytes contained in this input byte array  
            
        @return result                : parser result. 0 pass otherwise fail
        @return headerStartIndex      : the mmw demo output packet header start location
        @return totalPacketNumBytes   : the mmw demo output packet lenght           
        @return numDetObj             : the number of detected objects contained in this mmw demo output packet          
        @return numTlv                : the number of TLV contained in this mmw demo output packet           
        @return subFrameNumber        : the sbuframe index (0,1,2 or 3) of the frame contained in this mmw demo output packet
        @return detectedX_array       : 1-demension array holds each detected target's x of the mmw demo output packet
        @return detectedY_array       : 1-demension array holds each detected target's y of the mmw demo output packet
        @return detectedZ_array       : 1-demension array holds each detected target's z of the mmw demo output packet
        @return detectedV_array       : 1-demension array holds each detected target's v of the mmw demo output packet
        @return detectedRange_array   : 1-demension array holds each detected target's range profile of the mmw demo output packet
        @return detectedAzimuth_array : 1-demension array holds each detected target's azimuth of the mmw demo output packet
        @return detectedElevAngle_array : 1-demension array holds each detected target's elevAngle of the mmw demo output packet
        @return detectedSNR_array     : 1-demension array holds each detected target's snr of the mmw demo output packet
        @return detectedNoise_array   : 1-demension array holds each detected target's noise of the mmw demo output packet
    """

    headerNumBytes = 40   

    PI = 3.14159265

    detectedX_array = []
    detectedY_array = []
    detectedZ_array = []
    detectedV_array = []
    detectedRange_array = []
    detectedAzimuth_array = []
    detectedElevAngle_array = []
    detectedSNR_array = []
    detectedNoise_array = []

    result = TC_PASS

    # call parser_helper() function to find the output packet header start location and packet size 
    (headerStartIndex, totalPacketNumBytes, numDetObj, numTlv, subFrameNumber) = parser_helper(data, readNumBytes, debug)
                         
    if headerStartIndex == -1:
        result = TC_FAIL
        print("************ Frame Fail, cannot find the magic words *****************")
    else:
        nextHeaderStartIndex = headerStartIndex + totalPacketNumBytes 

        if headerStartIndex + totalPacketNumBytes > readNumBytes:
            result = TC_FAIL
            print("********** Frame Fail, readNumBytes may not long enough ***********")
        elif nextHeaderStartIndex + 8 < readNumBytes and checkMagicPattern(data[nextHeaderStartIndex:nextHeaderStartIndex+8:1]) == 0:
            result = TC_FAIL
            print("********** Frame Fail, incomplete packet **********") 
        elif numDetObj <= 0:
            result = TC_FAIL
            print("************ Frame Fail, numDetObj = %d *****************" % (numDetObj))
        elif subFrameNumber > 3:
            result = TC_FAIL
            print("************ Frame Fail, subFrameNumber = %d *****************" % (subFrameNumber))
        else: 
            # process the 1st TLV
            tlvStart = headerStartIndex + headerNumBytes
                                                    
            tlvType    = getUint32(data[tlvStart+0:tlvStart+4:1])
            tlvLen     = getUint32(data[tlvStart+4:tlvStart+8:1])       
            offset = 8
            if(debug):        
                print("The 1st TLV") 
                print("    type %d" % (tlvType))
                print("    len %d bytes" % (tlvLen))
                                                    
            # the 1st TLV must be type 1
            if tlvType == 1 and tlvLen < totalPacketNumBytes:#MMWDEMO_UART_MSG_DETECTED_POINTS
                         
                # TLV type 1 contains x, y, z, v values of all detect objects. 
                # each x, y, z, v are 32-bit float in IEEE 754 single-precision binary floating-point format, so every 16 bytes represent x, y, z, v values of one detect objects.    
                
                # for each detect objects, extract/convert float x, y, z, v values and calculate range profile and azimuth                           
                for obj in range(numDetObj):
                    # convert byte0 to byte3 to float x value
                    x = struct.unpack('<f', codecs.decode(binascii.hexlify(data[tlvStart + offset:tlvStart + offset+4:1]),'hex'))[0]

                    # convert byte4 to byte7 to float y value
                    y = struct.unpack('<f', codecs.decode(binascii.hexlify(data[tlvStart + offset+4:tlvStart + offset+8:1]),'hex'))[0]

                    # convert byte8 to byte11 to float z value
                    z = struct.unpack('<f', codecs.decode(binascii.hexlify(data[tlvStart + offset+8:tlvStart + offset+12:1]),'hex'))[0]

                    # convert byte12 to byte15 to float v value
                    v = struct.unpack('<f', codecs.decode(binascii.hexlify(data[tlvStart + offset+12:tlvStart + offset+16:1]),'hex'))[0]

                    # calculate range profile from x, y, z
                    compDetectedRange = math.sqrt((x * x)+(y * y)+(z * z))

                    # calculate azimuth from x, y           
                    if y == 0:
                        if x >= 0:
                            detectedAzimuth = 90
                        else:
                            detectedAzimuth = -90 
                    else:
                        detectedAzimuth = math.atan(x/y) * 180 / PI

                    # calculate elevation angle from x, y, z
                    if x == 0 and y == 0:
                        if z >= 0:
                            detectedElevAngle = 90
                        else: 
                            detectedElevAngle = -90
                    else:
                        detectedElevAngle = math.atan(z/math.sqrt((x * x)+(y * y))) * 180 / PI
                            
                    detectedX_array.append(x)
                    detectedY_array.append(y)
                    detectedZ_array.append(z)
                    detectedV_array.append(v)
                    detectedRange_array.append(compDetectedRange)
                    detectedAzimuth_array.append(detectedAzimuth)
                    detectedElevAngle_array.append(detectedElevAngle)
                                                                
                    offset = offset + 16
                # end of for obj in range(numDetObj) for 1st TLV
                                                            
            # Process the 2nd TLV
            tlvStart = tlvStart + 8 + tlvLen
                                                    
            tlvType    = getUint32(data[tlvStart+0:tlvStart+4:1])
            tlvLen     = getUint32(data[tlvStart+4:tlvStart+8:1])      
            offset = 8

            if(debug):        
                print("The 2nd TLV") 
                print("    type %d" % (tlvType))
                print("    len %d bytes" % (tlvLen))
                                                            
            if tlvType == 7: 
                
                # TLV type 7 contains snr and noise of all detect objects.
                # each snr and noise are 16-bit integer represented by 2 bytes, so every 4 bytes represent snr and noise of one detect objects.    
            
                # for each detect objects, extract snr and noise                                            
                for obj in range(numDetObj):
                    # byte0 and byte1 represent snr. convert 2 bytes to 16-bit integer
                    snr   = getUint16(data[tlvStart + offset + 0:tlvStart + offset + 2:1])
                    # byte2 and byte3 represent noise. convert 2 bytes to 16-bit integer 
                    noise = getUint16(data[tlvStart + offset + 2:tlvStart + offset + 4:1])

                    detectedSNR_array.append(snr)
                    detectedNoise_array.append(noise)
                                                                    
                    offset = offset + 4
            else:
                for obj in range(numDetObj):
                    detectedSNR_array.append(0)
                    detectedNoise_array.append(0)
            # end of if tlvType == 7
            if(debug):           
                print("                  x(m)         y(m)         z(m)        v(m/s)    Com0range(m)  azimuth(deg)  elevAngle(deg)  snr(0.1dB)    noise(0.1dB)")
                for obj in range(numDetObj):
                    print("    obj%3d: %12f %12f %12f %12f %12f %12f %12d %12d %12d" % (obj, detectedX_array[obj], detectedY_array[obj], detectedZ_array[obj], detectedV_array[obj], detectedRange_array[obj], detectedAzimuth_array[obj], detectedElevAngle_array[obj], detectedSNR_array[obj], detectedNoise_array[obj]))
                
    return (result, headerStartIndex, totalPacketNumBytes, numDetObj, numTlv, subFrameNumber, detectedX_array, detectedY_array, detectedZ_array, detectedV_array, detectedRange_array, detectedAzimuth_array, detectedElevAngle_array, detectedSNR_array, detectedNoise_array)


# Function to configure the serial ports and send the data from
# the configuration file to the radar
def serialConfig(configFileName):
    
    global CLIport
    global Dataport
    # Open the serial ports for the configuration and the data ports
    
    # Raspberry pi
    CLIport = serial.Serial('/dev/ttyUSB0', 115200)
    Dataport = serial.Serial('/dev/ttyUSB1', 921600)


    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        CLIport.write((i+'\n').encode())
        print(i)
        time.sleep(0.01)
        
    return CLIport, Dataport

# Function to parse the data inside the configuration file
def parseConfigFile(configFileName):
    configParameters = {} # Initialize an empty dictionary to store the configuration parameters
    
    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        
        # Split the line
        splitWords = i.split(" ")
        
        # Hard code the number of antennas, change if other configuration is used
        numRxAnt = 4
        numTxAnt = 3
        
        # Get the information about the profile configuration
        if "profileCfg" in splitWords[0]:
            startFreq = int(float(splitWords[2]))
            idleTime = int(splitWords[3])
            rampEndTime = float(splitWords[5])
            freqSlopeConst = float(splitWords[8])
            numAdcSamples = int(splitWords[10])
            numAdcSamplesRoundTo2 = 1
            
            while numAdcSamples > numAdcSamplesRoundTo2:
                numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2
                
            digOutSampleRate = int(splitWords[11])
            
        # Get the information about the frame configuration    
        elif "frameCfg" in splitWords[0]:
            
            chirpStartIdx = int(splitWords[1])
            chirpEndIdx = int(splitWords[2])
            numLoops = int(splitWords[3])
            # numFrames = int(splitWords[4])
            # framePeriodicity = int(splitWords[5])

            
    # Combine the read data to obtain the configuration parameters           
    numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    configParameters["numDopplerBins"] = numChirpsPerFrame / numTxAnt
    configParameters["numRangeBins"] = numAdcSamplesRoundTo2
    configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * numAdcSamples)
    configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"])
    configParameters["dopplerResolutionMps"] = 3e8 / (2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * configParameters["numDopplerBins"] * numTxAnt)
    configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate)/(2 * freqSlopeConst * 1e3)
    configParameters["maxVelocity"] = 3e8 / (4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt)
    
    return configParameters

##################################################################################
# USE parser_mmw_demo SCRIPT TO PARSE ABOVE INPUT FILES
##################################################################################
def readAndParseData14xx(Dataport, configParameters):
    #load from serial
    global byteBuffer, byteBufferLength

    # Initialize variables
    magicOK = 0 # Checks if magic number has been read
    dataOK = 0 # Checks if the data has been read correctly
    frameNumber = 0
    detObj = {}

    readBuffer = Dataport.read(Dataport.in_waiting)
    byteVec = np.frombuffer(readBuffer, dtype = 'uint8')
    byteCount = len(byteVec)

    # Check that the buffer is not full, and then add the data to the buffer
    if (byteBufferLength + byteCount) < maxBufferSize:
        byteBuffer[byteBufferLength:byteBufferLength + byteCount] = byteVec[:byteCount]
        byteBufferLength = byteBufferLength + byteCount
    
    # Check that the buffer has some data
    if byteBufferLength > 16:
        
        # Check for all possible locations of the magic word
        possibleLocs = np.where(byteBuffer == magicWord[0])[0]

        # Confirm that is the beginning of the magic word and store the index in startIdx
        startIdx = []
        for loc in possibleLocs:
            check = byteBuffer[loc:loc+8]
            if np.all(check == magicWord):
                startIdx.append(loc)

        # Check that startIdx is not empty
        if startIdx:
            
            # Remove the data before the first start index
            if startIdx[0] > 0 and startIdx[0] < byteBufferLength:
                byteBuffer[:byteBufferLength-startIdx[0]] = byteBuffer[startIdx[0]:byteBufferLength]
                byteBuffer[byteBufferLength-startIdx[0]:] = np.zeros(len(byteBuffer[byteBufferLength-startIdx[0]:]),dtype = 'uint8')
                byteBufferLength = byteBufferLength - startIdx[0]
                
            # Check that there have no errors with the byte buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0

            # Read the total packet length
            totalPacketLen = np.matmul(byteBuffer[12:12+4],word)
            # Check that all the packet has been read
            if (byteBufferLength >= totalPacketLen) and (byteBufferLength != 0):
                magicOK = 1
    
    # If magicOK is equal to 1 then process the message
    if magicOK:
        # Read the entire buffer
        readNumBytes = byteBufferLength
        if(DEBUG):
            print("readNumBytes: ", readNumBytes)
        allBinData = byteBuffer
        if(DEBUG):
            print("allBinData: ", allBinData[0], allBinData[1], allBinData[2], allBinData[3])

        # init local variables
        totalBytesParsed = 0
        numFramesParsed = 0

        # parser_one_mmw_demo_output_packet extracts only one complete frame at a time
        # so call this in a loop till end of file
        #             
        # parser_one_mmw_demo_output_packet function already prints the
        # parsed data to stdio. So showcasing only saving the data to arrays 
        # here for further custom processing
        parser_result, \
        headerStartIndex,  \
        totalPacketNumBytes, \
        numDetObj,  \
        numTlv,  \
        subFrameNumber,  \
        detectedX_array,  \
        detectedY_array,  \
        detectedZ_array,  \
        detectedV_array,  \
        detectedRange_array,  \
        detectedAzimuth_array,  \
        detectedElevation_array,  \
        detectedSNR_array,  \
        detectedNoise_array = parser_one_mmw_demo_output_packet(allBinData[totalBytesParsed::1], readNumBytes-totalBytesParsed,DEBUG)

        # Check the parser result
        if(DEBUG):
            print ("Parser result: ", parser_result)
        if (parser_result == 0): 
            totalBytesParsed += (headerStartIndex+totalPacketNumBytes)    
            numFramesParsed+=1
            if(DEBUG):
                print("totalBytesParsed: ", totalBytesParsed)
            ##################################################################################
            # TODO: use the arrays returned by above parser as needed. 
            # For array dimensions, see help(parser_one_mmw_demo_output_packet)
            # help(parser_one_mmw_demo_output_packet)
            ##################################################################################

        
            detObj = {"numObj": numDetObj, "range": detectedRange_array, \
                        "x": detectedX_array, "y": detectedY_array, "z": detectedZ_array}
            dataOK = 1 
        else: 
            # error in parsing; exit the loop
            print("error in parsing this frame; continue")

        
        shiftSize = totalPacketNumBytes            
        byteBuffer[:byteBufferLength - shiftSize] = byteBuffer[shiftSize:byteBufferLength]
        byteBuffer[byteBufferLength - shiftSize:] = np.zeros(len(byteBuffer[byteBufferLength - shiftSize:]),dtype = 'uint8')
        byteBufferLength = byteBufferLength - shiftSize
        
        # Check that there are no errors with the buffer length
        if byteBufferLength < 0:
            byteBufferLength = 0
        # All processing done; Exit
        if(DEBUG):
            print("numFramesParsed: ", numFramesParsed)

    if dataOK:
        print(detObj['numObj'])

    return dataOK, frameNumber, detObj



def main():        
    # Configurate the serial port
    CLIport, Dataport = serialConfig(configFileName)

    # Get the configuration parameters from the configuration file
    global configParameters 
    configParameters = parseConfigFile(configFileName)

    while 1:
        readAndParseData14xx(Dataport, configParameters)

    CLIport.write(('sensorStop\n').encode())
    CLIport.close()
    Dataport.close()



if __name__ == "__main__":
    main()

