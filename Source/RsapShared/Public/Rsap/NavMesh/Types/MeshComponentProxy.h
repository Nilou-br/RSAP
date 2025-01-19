// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once



struct FRsapMeshComponentRenderData
{
	// FBufferRHIRef PrefixSumBuffer;
	// FShaderResourceViewRHIRef  PrefixSumBufferSRV;
	// FUnorderedAccessViewRHIRef PrefixSumBufferUAV;

	TRefCountPtr<FRDGPooledBuffer> PrefixSumBuffer;
};
