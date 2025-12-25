#!/usr/bin/env bash
# Download ML model artifacts
# Reads from files/artifacts.yaml manifest

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DATA_DIR="${DATA_DIR:-$(dirname "$SCRIPT_DIR")/../../data}"
MANIFEST="${SCRIPT_DIR}/../files/artifacts.yaml"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m'

echo "Downloading ML model artifacts..."
echo "  Data directory: ${DATA_DIR}"
echo "  Manifest: ${MANIFEST}"
echo ""

# Check if yq is available for YAML parsing, otherwise use simple parsing
if command -v yq &> /dev/null; then
    USE_YQ=true
else
    USE_YQ=false
    echo -e "${YELLOW}Note: yq not found, using simple parser (install yq for better YAML support)${NC}"
fi

# Track statistics
DOWNLOADED=0
SKIPPED=0
FAILED=0

download_file() {
    local dir="$1"
    local name="$2"
    local url="$3"
    local sha256="$4"
    local extract="$5"
    local extract_dir="$6"

    local dest_dir="${DATA_DIR}/${dir}"
    local dest_file="${dest_dir}/${name}"

    mkdir -p "${dest_dir}"

    # Check if file exists and verify checksum
    if [[ -f "${dest_file}" ]] && [[ -n "${sha256}" ]]; then
        local actual_sha256
        actual_sha256=$(sha256sum "${dest_file}" | cut -d' ' -f1)
        if [[ "${actual_sha256}" == "${sha256}" ]]; then
            printf "${GREEN}✓${NC} %s/%s (cached)\n" "${dir}" "${name}"
            ((SKIPPED++)) || true
            return 0
        fi
    elif [[ -f "${dest_file}" ]] && [[ -z "${sha256}" ]]; then
        printf "${GREEN}✓${NC} %s/%s (exists, no checksum)\n" "${dir}" "${name}"
        ((SKIPPED++)) || true
        return 0
    fi

    # Download
    printf "${YELLOW}↓${NC} %s/%s\n" "${dir}" "${name}"
    if ! curl -fSL --progress-bar -o "${dest_file}" "${url}"; then
        printf "${RED}✗${NC} Failed to download %s/%s\n" "${dir}" "${name}"
        ((FAILED++)) || true
        return 1
    fi

    # Verify checksum
    if [[ -n "${sha256}" ]]; then
        local actual_sha256
        actual_sha256=$(sha256sum "${dest_file}" | cut -d' ' -f1)
        if [[ "${actual_sha256}" != "${sha256}" ]]; then
            printf "${RED}✗${NC} Checksum mismatch for %s/%s\n" "${dir}" "${name}"
            rm -f "${dest_file}"
            ((FAILED++)) || true
            return 1
        fi
    fi

    # Extract if needed
    if [[ "${extract}" == "true" ]]; then
        local target_dir="${dest_dir}"
        [[ -n "${extract_dir}" ]] && target_dir="${dest_dir}/${extract_dir}"
        mkdir -p "${target_dir}"

        printf "  Extracting to %s...\n" "${target_dir}"
        case "${name}" in
            *.tar.gz|*.tgz)
                tar -xzf "${dest_file}" -C "${target_dir}"
                ;;
            *.tar)
                tar -xf "${dest_file}" -C "${target_dir}"
                ;;
            *.zip)
                unzip -q -o "${dest_file}" -d "${target_dir}"
                ;;
            *)
                printf "${YELLOW}!${NC} Unknown archive format: %s\n" "${name}"
                ;;
        esac
    fi

    ((DOWNLOADED++)) || true
    return 0
}

# Simple YAML parser (handles our specific format)
parse_and_download() {
    local current_dir=""
    local name="" url="" sha256="" extract="" extract_dir=""

    while IFS= read -r line || [[ -n "$line" ]]; do
        # Skip comments and empty lines
        [[ "$line" =~ ^[[:space:]]*# ]] && continue
        [[ -z "${line// }" ]] && continue

        # Directory header (no leading whitespace, ends with :)
        if [[ "$line" =~ ^[a-zA-Z_][a-zA-Z0-9_-]*:$ ]]; then
            current_dir="${line%:}"
            continue
        fi

        # Item start (- name:)
        if [[ "$line" =~ ^[[:space:]]*-[[:space:]]*name:[[:space:]]*(.+)$ ]]; then
            # Download previous item if exists
            if [[ -n "$name" ]] && [[ -n "$url" ]]; then
                download_file "$current_dir" "$name" "$url" "$sha256" "$extract" "$extract_dir"
            fi
            name="${BASH_REMATCH[1]}"
            url="" sha256="" extract="" extract_dir=""
            continue
        fi

        # Properties
        if [[ "$line" =~ ^[[:space:]]*url:[[:space:]]*(.+)$ ]]; then
            url="${BASH_REMATCH[1]}"
        elif [[ "$line" =~ ^[[:space:]]*sha256:[[:space:]]*(.+)$ ]]; then
            sha256="${BASH_REMATCH[1]}"
        elif [[ "$line" =~ ^[[:space:]]*extract:[[:space:]]*(.+)$ ]]; then
            extract="${BASH_REMATCH[1]}"
        elif [[ "$line" =~ ^[[:space:]]*extract_dir:[[:space:]]*(.+)$ ]]; then
            extract_dir="${BASH_REMATCH[1]}"
        fi
    done < "$MANIFEST"

    # Download last item
    if [[ -n "$name" ]] && [[ -n "$url" ]]; then
        download_file "$current_dir" "$name" "$url" "$sha256" "$extract" "$extract_dir"
    fi
}

# Run the parser
parse_and_download

echo ""
echo "========================================"
echo "Download Summary:"
echo "  Downloaded: ${DOWNLOADED}"
echo "  Skipped (cached): ${SKIPPED}"
echo "  Failed: ${FAILED}"
echo "========================================"

if [[ ${FAILED} -gt 0 ]]; then
    echo -e "${RED}Some downloads failed. Re-run to retry.${NC}"
    exit 1
fi

echo -e "${GREEN}All artifacts downloaded successfully.${NC}"
