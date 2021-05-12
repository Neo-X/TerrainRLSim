// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2016
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 2.1.0 (2016/01/25)

#pragma once

#include <Graphics/GteBuffer.h>
#include <Graphics/DX11/GteDX11Resource.h>

namespace gte
{

class GTE_IMPEXP DX11Buffer : public DX11Resource
{
protected:
    // Abstract base class.
    DX11Buffer(Buffer const* buffer);

public:
    // Member access.
    inline Buffer* GetBuffer() const;
    inline ID3D11Buffer* GetDXBuffer() const;

    // Copy data from CPU to GPU via mapped memory.  Buffers use only
    // subresource 0, so the subresource index (sri) is not exposed.
    virtual bool Update(ID3D11DeviceContext* context) override;
    virtual bool CopyCpuToGpu(ID3D11DeviceContext* context) override;
    virtual bool CopyGpuToCpu(ID3D11DeviceContext* context) override;

private:
    // Buffers use only subresource 0, so these overrides are stubbed out.
    virtual bool Update(ID3D11DeviceContext* context, unsigned int sri) override;
    virtual bool CopyCpuToGpu(ID3D11DeviceContext* context, unsigned int sri) override;
    virtual bool CopyGpuToCpu(ID3D11DeviceContext* context, unsigned int sri) override;

protected:
    // Support for creating staging buffers.
    void CreateStaging(ID3D11Device* device, D3D11_BUFFER_DESC const& bf);
};

inline Buffer* DX11Buffer::GetBuffer() const
{
    return static_cast<Buffer*>(mGTObject);
}

inline ID3D11Buffer* DX11Buffer::GetDXBuffer() const
{
    return static_cast<ID3D11Buffer*>(mDXObject);
}

}
