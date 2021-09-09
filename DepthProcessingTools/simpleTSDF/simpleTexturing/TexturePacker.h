#pragma once

#pragma once
#include "MeshUtils.h"
#include <uvpCore.hpp>

#include <iostream>
#include <unordered_map>
#include <list>
#include <vector>
#include <string>
#include <cstring>
#include <array>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <opencv2/opencv.hpp>
//#include "open3d/Open3D.h"
//#include "open3d/io/sensor/azure_kinect/MKVMetadata.h"
//#include "open3d/geometry/RGBDImage.h"
//#include "ErrorLogger.h"

using namespace uvpcore;
//using namespace open3d;

//TODO: Make sense of all this stuff
inline void opExecutorMessageHandler(void* m_pMessageHandlerData, UvpMessageT* pMsg);

class TexturePacker
{
    friend void opExecutorMessageHandler(void* m_pMessageHandlerData, UvpMessageT* pMsg);

    typedef std::array<UvpMessageT*, static_cast<int>(UvpMessageT::MESSAGE_CODE::VALUE_COUNT)> UvpMessageArrayT;

    class UvpOpExecutorT
    {
    private:
        friend void opExecutorMessageHandler(void* m_pMessageHandlerData, UvpMessageT* pMsg);

        std::list<UvpMessageT*> m_ReceivedMessages;
        UvpMessageArrayT m_LastMessagePerCode;

        bool m_DebugMode;

        void destroyMessages()
        {
            // The application becomes the owner of UVP messages after receiving it,
            // so we have to make sure they are eventually deallocated by calling
            // the destory method on them (do not use the delete operator).
            for (UvpMessageT* pMsg : m_ReceivedMessages)
            {
                pMsg->destroy();
            }
            m_ReceivedMessages.clear();
        }

        void reset()
        {
            destroyMessages();
            m_LastMessagePerCode = { nullptr };
        }

        void handleMessage(UvpMessageT* pMsg)
        {
            // This method is called every time the packer sends a message to the application.
            // We need to handle the message properly.

            if (pMsg->m_Code == UvpMessageT::MESSAGE_CODE::PROGRESS_REPORT)
            {
                UvpProgressReportMessageT* pReportProgressMsg = static_cast<UvpProgressReportMessageT*>(pMsg);

                std::cout << "[UVP PROGRESS REPORT] Phase: " << static_cast<int>(pReportProgressMsg->m_PackingPhase);
                for (int i = 0; i < pReportProgressMsg->m_ProgressSize; i++)
                {
                    std::cout << ", Progress[" << i << "]: " << pReportProgressMsg->m_ProgressArray[i];
                }
                std::cout << "\n";
            }
            else if (pMsg->m_Code == UvpMessageT::MESSAGE_CODE::BENCHMARK)
            {
                UvpBenchmarkMessageT* pBenchmarkMsg = static_cast<UvpBenchmarkMessageT*>(pMsg);

                std::cout << "[UVP BENCHMARK] Device name: " << pBenchmarkMsg->m_DeviceName.c_str() << ", Total packing time (ms): " <<
                    pBenchmarkMsg->m_TotalPackTimeMs << ", Average packing time (ms): " << pBenchmarkMsg->m_AvgPackTimeMs << "\n";
            }

            m_LastMessagePerCode[static_cast<int>(pMsg->m_Code)] = pMsg;
            m_ReceivedMessages.push_back(pMsg);
        }


    public:
        UvpOpExecutorT(bool debugMode) :
            m_DebugMode(debugMode)
        {}

        ~UvpOpExecutorT()
        {
            destroyMessages();
        }

        UVP_ERRORCODE execute(UvpOperationInputT& uvpInput)
        {
            reset();

            uvpInput.m_pMessageHandler = opExecutorMessageHandler;
            uvpInput.m_pMessageHandlerData = this;

            if (m_DebugMode)
            {
                // Check whether the application configurated the operation input properly.
                // WARNING: this operation is time consuming (in particular it iterates over all UV data),
                // that is why it should only be executed when debugging the application. It should
                // never be used in production.
                const char* pValidationResult = uvpInput.validate();

                if (pValidationResult)
                {
                   // E_LOG("Operation input validation failed: " + std::string(pValidationResult), true);
                }
            }

            UvpOperationT uvpOp(uvpInput);
            // Execute the actual operation - this call will block the current thread
            // until the operation is finished.
            UVP_ERRORCODE retCode = uvpOp.entry();

            return retCode;
        }

        UvpMessageT* getLastMessage(UvpMessageT::MESSAGE_CODE code)
        {
            return m_LastMessagePerCode[static_cast<int>(code)];
        }
    };

    void islandSolutionToMatrix(const UvpIslandPackSolutionT& islandSolution, Eigen::Matrix4d& mat)
    {
        // Generate matrix used to transform UVs of the given island in order to apply
        // a packing result
        mat = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d rotZmat = Eigen::Matrix4d::Identity();

        Eigen::Vector3d translate_in_place(islandSolution.m_PostScaleOffset[0], islandSolution.m_PostScaleOffset[1], 0.0f);

        mat.block<3, 1>(0, 3) += mat.block<3, 3>(0, 0) * translate_in_place;

        mat.block<3, 1>(0, 0) *= 1.0 / islandSolution.m_Scale;
        mat.block<3, 1>(0, 1) *= 1.0 / islandSolution.m_Scale;

        translate_in_place = Eigen::Vector3d(islandSolution.m_Offset[0], islandSolution.m_Offset[1], 0.0f);
        mat.block<3, 1>(0, 3) += mat.block<3, 3>(0, 0) * translate_in_place;

        translate_in_place = Eigen::Vector3d(islandSolution.m_Pivot[0], islandSolution.m_Pivot[1], 0.0f);
        mat.block<3, 1>(0, 3) += mat.block<3, 3>(0, 0) * translate_in_place;

        auto ca = cos(islandSolution.m_Angle);
        auto sa = sin(islandSolution.m_Angle);

        rotZmat.block<1, 1>(0, 0)[0] = ca;
        rotZmat.block<1, 1>(1, 1)[0] = ca;
        rotZmat.block<1, 1>(0, 1)[0] = -sa;
        rotZmat.block<1, 1>(1, 0)[0] = sa;

        mat = mat * rotZmat;

        mat.block<3, 1>(0, 3) -= mat.block<3, 3>(0, 0) * translate_in_place;

        mat.block<3, 1>(0, 0) *= islandSolution.m_PreScale;
        mat.block<3, 1>(0, 1) *= islandSolution.m_PreScale;

        //mat4x4_identity(mat);
        //mat4x4_translate_in_place(mat, islandSolution.m_PostScaleOffset[0], islandSolution.m_PostScaleOffset[1], 0.0);
        //mat4x4_scale_aniso(mat, mat, 1.0 / islandSolution.m_Scale, 1.0 / islandSolution.m_Scale, 1.0);
        //mat4x4_translate_in_place(mat, islandSolution.m_Offset[0], islandSolution.m_Offset[1], 0.0);
        //mat4x4_translate_in_place(mat, islandSolution.m_Pivot[0], islandSolution.m_Pivot[1], 0.0);
        //mat4x4_rotate_Z(mat, mat, islandSolution.m_Angle);
        //mat4x4_translate_in_place(mat, -islandSolution.m_Pivot[0], -islandSolution.m_Pivot[1], 0.0);
        //mat4x4_scale_aniso(mat, mat, islandSolution.m_PreScale, islandSolution.m_PreScale, 1.0);
    }

public:
    bool PackUV(bool debug);//geometry::Image& im, geometry::TriangleMesh& mesh, bool debug);

    bool PerformTexturePack(MyMesh& mesh, std::vector<cv::Mat>& color_array, cv::Mat& outputImage, bool debug_info);// std::vector<open3d::geometry::Image>* color_array, geometry::TriangleMesh* mesh, geometry::Image* outputImage, bool debug_info);
};

inline void opExecutorMessageHandler(void* m_pMessageHandlerData, UvpMessageT* pMsg)
{
    reinterpret_cast<TexturePacker::UvpOpExecutorT*>(m_pMessageHandlerData)->handleMessage(pMsg);
}