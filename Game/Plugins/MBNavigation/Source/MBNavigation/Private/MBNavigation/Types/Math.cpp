// Copyright Melvin Brink 2023. All Rights Reserved.

#include "MBNavigation/Types/Math.h"


template <typename VectorType>
bool TBounds<VectorType>::HasOverlap(const UWorld* World) const
{
	static_assert(std::is_same_v<VectorType, F3DVector32>, "TBounds::HasOverlap is only supported for F3DVector32");
	
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("TBounds Has-Overlap");
	return FPhysicsInterface::GeomOverlapBlockingTest(
		World,
		FCollisionShape::MakeBox(GetExtents().ToVector()),
		GetCenter().ToVector(),
		FQuat::Identity,
		ECollisionChannel::ECC_WorldStatic,
		FCollisionQueryParams::DefaultQueryParam,
		FCollisionResponseParams::DefaultResponseParam
	);
}