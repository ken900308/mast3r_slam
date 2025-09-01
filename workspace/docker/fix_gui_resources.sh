#!/bin/bash
# Script to fix GUI resources for MASt3R-SLAM visualization

echo "=== Fixing GUI Resources ==="

# Create the resources directory if it doesn't exist
echo "Creating resources directory..."
mkdir -p /usr/local/lib/python3.10/dist-packages/resources

# Copy in3d resources (fonts, meshes, shader programs)
echo "Copying in3d resources..."
if [ -d "/workspace/MASt3R-SLAM/thirdparty/in3d/resources" ]; then
    cp -r /workspace/MASt3R-SLAM/thirdparty/in3d/resources/* /usr/local/lib/python3.10/dist-packages/resources/ 2>/dev/null || true
    echo "  ✓ in3d resources copied"
else
    echo "  ⚠ in3d resources not found"
fi

# Copy MASt3R-SLAM resources (including lines.glsl shader)
echo "Copying MASt3R-SLAM resources..."
if [ -d "/workspace/MASt3R-SLAM/resources" ]; then
    cp -r /workspace/MASt3R-SLAM/resources/* /usr/local/lib/python3.10/dist-packages/resources/ 2>/dev/null || true
    echo "  ✓ MASt3R-SLAM resources copied"
else
    echo "  ⚠ MASt3R-SLAM resources not found"
fi

# Verify the resources are in place
echo "Verifying resources..."
if [ -d "/usr/local/lib/python3.10/dist-packages/resources" ]; then
    RESOURCE_COUNT=$(find /usr/local/lib/python3.10/dist-packages/resources -type f | wc -l)
    echo "  ✓ Resources directory created with $RESOURCE_COUNT files"
    echo "  Available resource types:"
    ls -la /usr/local/lib/python3.10/dist-packages/resources/ | grep -E "^d" | awk '{print "    - " $9}' | grep -v "^\s*- \.$" | grep -v "^\s*- \.\.$"
else
    echo "  ❌ Resources directory creation failed"
    exit 1
fi

echo "=== GUI Resources Fix Complete ==="
