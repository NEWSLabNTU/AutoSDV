#!/bin/bash
#
# Synology Drive Public Link Downloader
# Downloads files from Synology Drive public sharing links
#
# Usage: ./synology-drive-download.sh [options] <share_url> [output_dir]
#
# Example:
#   ./synology-drive-download.sh "https://example.com/drive/d/s/ABC123/XYZ789" ./downloads
#

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Default settings
OUTPUT_DIR="."
INFO_ONLY=false
FORCE_OVERWRITE=false

usage() {
    echo "Synology Drive Public Link Downloader"
    echo ""
    echo "Usage: $0 [options] <share_url> [output_dir]"
    echo ""
    echo "Arguments:"
    echo "  share_url    Synology Drive public sharing URL"
    echo "  output_dir   Output directory (default: current directory)"
    echo ""
    echo "Options:"
    echo "  -h, --help     Show this help message"
    echo "  -i, --info     Show file info only, don't download"
    echo "  -f, --force    Force overwrite without prompting"
    echo ""
    echo "Example:"
    echo "  $0 'https://nas.example.com/drive/d/s/ABC123/XYZ789' ./downloads"
    echo "  $0 --info 'https://nas.example.com/drive/d/s/ABC123/XYZ789'"
    exit 0
}

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check dependencies
check_dependencies() {
    local missing=()
    for cmd in curl python3; do
        if ! command -v "$cmd" &> /dev/null; then
            missing+=("$cmd")
        fi
    done

    if [ ${#missing[@]} -ne 0 ]; then
        log_error "Missing dependencies: ${missing[*]}"
        log_error "Please install: ${missing[*]}"
        exit 1
    fi
}

# Parse the share URL to extract components
parse_share_url() {
    local url="$1"

    # Expected format: https://host/drive/d/s/PERMANENT_LINK/SHARING_LINK
    if [[ ! "$url" =~ ^https?://([^/]+)/drive/d/s/([^/]+)/([^/?]+) ]]; then
        log_error "Invalid Synology Drive share URL format"
        log_error "Expected: https://host/drive/d/s/PERMANENT_LINK/SHARING_LINK"
        exit 1
    fi

    HOST="${BASH_REMATCH[1]}"
    PERMANENT_LINK="${BASH_REMATCH[2]}"
    SHARING_LINK="${BASH_REMATCH[3]}"
    BASE_URL="https://${HOST}"

    log_info "Host: $HOST"
    log_info "Permanent Link: $PERMANENT_LINK"
    log_info "Sharing Link: $SHARING_LINK"
}

# Get session cookie from the share page
get_session_cookie() {
    local share_url="$1"

    log_info "Getting session cookie..."

    # Get cookie from response headers
    local headers
    headers=$(curl -sI "$share_url" 2>&1)

    # Extract set-cookie header for the sharing cookie
    COOKIE=$(echo "$headers" | grep -i "set-cookie:" | grep "drive-sharing-" | head -1 | sed 's/[Ss]et-[Cc]ookie: //' | cut -d';' -f1)

    if [ -z "$COOKIE" ]; then
        log_error "Failed to get session cookie"
        exit 1
    fi

    COOKIE_VALUE=$(echo "$COOKIE" | cut -d'=' -f2)
    log_info "Session cookie obtained (${#COOKIE_VALUE} chars)"
}

# Get file metadata from the getjs API
get_file_metadata() {
    log_info "Getting file metadata..."

    # URL encode the parameters
    local encoded_perm=$(python3 -c "import urllib.parse; print(urllib.parse.quote('\"$PERMANENT_LINK\"'))")
    local encoded_share=$(python3 -c "import urllib.parse; print(urllib.parse.quote('\"$SHARING_LINK\"'))")

    local api_url="${BASE_URL}/drive/webapi/entry.cgi?api=SYNO.SynologyDrive.Shard&version=1&method=getjs&permanent_link=${encoded_perm}&sharing_link=${encoded_share}"

    # Get the JavaScript response and save to temp file to avoid escaping issues
    local tmp_js=$(mktemp)
    # Clean up temp file on exit (append to any existing trap)
    trap "rm -f '$tmp_js'" EXIT

    curl -s "$api_url" > "$tmp_js" 2>&1

    # Use Python to extract and parse the JSON from the JavaScript response
    local metadata
    metadata=$(python3 - "$tmp_js" << 'PYEOF'
import re
import json
import sys

if len(sys.argv) < 2:
    print("ERROR: No input file", file=sys.stderr)
    sys.exit(1)

with open(sys.argv[1], 'r') as f:
    js_content = f.read()

try:
    # Find getDriveFile function and extract the JSON object
    # The pattern is: getDriveFile=function(){return {...}}
    start_marker = 'getDriveFile=function(){return '
    start = js_content.find(start_marker)
    if start == -1:
        # Try with window. prefix
        start_marker = 'window.getDriveFile=function(){return '
        start = js_content.find(start_marker)

    if start == -1:
        print("ERROR: Could not find getDriveFile function", file=sys.stderr)
        sys.exit(1)

    # Find the start of JSON object
    json_start = start + len(start_marker)

    # Find matching closing brace by counting
    depth = 0
    json_end = json_start
    for i, c in enumerate(js_content[json_start:]):
        if c == '{':
            depth += 1
        elif c == '}':
            depth -= 1
            if depth == 0:
                json_end = json_start + i + 1
                break

    json_str = js_content[json_start:json_end]
    data = json.loads(json_str)

    file_id = data.get('file_id', '')
    file_name = data.get('name', 'download')
    file_size = data.get('size', 0)

    if not file_id:
        print("ERROR: No file_id in metadata", file=sys.stderr)
        sys.exit(1)

    # Output as tab-separated for safe parsing
    print(f"{file_id}\t{file_name}\t{file_size}")

except json.JSONDecodeError as e:
    print(f"ERROR: Failed to parse JSON: {e}", file=sys.stderr)
    sys.exit(1)
except Exception as e:
    print(f"ERROR: {e}", file=sys.stderr)
    sys.exit(1)
PYEOF
)

    if [ $? -ne 0 ] || [ -z "$metadata" ]; then
        log_error "Failed to extract file metadata"
        log_error "The share link may be invalid or expired"
        exit 1
    fi

    # Parse tab-separated output
    FILE_ID=$(echo "$metadata" | cut -f1)
    FILE_NAME=$(echo "$metadata" | cut -f2)
    FILE_SIZE=$(echo "$metadata" | cut -f3)

    log_info "File: $FILE_NAME"
    log_info "File ID: $FILE_ID"
    log_info "Size: $(numfmt --to=iec-i --suffix=B $FILE_SIZE 2>/dev/null || echo "${FILE_SIZE} bytes")"
}

# Construct download URL and download the file
download_file() {
    log_info "Preparing download..."

    # URL encode the sharing token (cookie value) and filename
    local encoded_token=$(python3 -c "import urllib.parse; print(urllib.parse.quote('\"$COOKIE_VALUE\"'))")
    local encoded_files=$(python3 -c "import urllib.parse; print(urllib.parse.quote('[\"id:$FILE_ID\"]'))")
    local encoded_filename=$(python3 -c "import urllib.parse; print(urllib.parse.quote('''$FILE_NAME'''))")

    # Construct download URL
    local download_url="${BASE_URL}/drive/d/s/${PERMANENT_LINK}/webapi/entry.cgi/SYNO.SynologyDrive.Files/${encoded_filename}?api=SYNO.SynologyDrive.Files&method=download&version=2&download_type=%22download%22&files=${encoded_files}&force_download=true&json_error=true&sharing_token=${encoded_token}"

    # Create output directory if needed
    mkdir -p "$OUTPUT_DIR"

    local output_path="${OUTPUT_DIR}/${FILE_NAME}"

    # Check if file already exists
    if [ -f "$output_path" ]; then
        if [ "$FORCE_OVERWRITE" = true ]; then
            log_warn "Overwriting existing file: $output_path"
        else
            log_warn "File already exists: $output_path"
            read -p "Overwrite? [y/N] " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                log_info "Download cancelled"
                exit 0
            fi
        fi
    fi

    log_info "Downloading to: $output_path"
    log_info "This may take a while for large files..."

    # Download with progress bar
    if curl -# -L -b "$COOKIE" -o "$output_path" "$download_url"; then
        log_info "Download complete: $output_path"

        # Verify file size
        local actual_size
        actual_size=$(stat -c%s "$output_path" 2>/dev/null || stat -f%z "$output_path" 2>/dev/null || echo "0")

        if [ "$actual_size" -eq "$FILE_SIZE" ]; then
            log_info "File size verified: $(numfmt --to=iec-i --suffix=B $actual_size 2>/dev/null || echo "${actual_size} bytes")"
        else
            log_warn "Size mismatch: expected $FILE_SIZE, got $actual_size"
        fi
    else
        log_error "Download failed"
        rm -f "$output_path"
        exit 1
    fi
}

# Parse command line arguments
parse_args() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -h|--help)
                usage
                ;;
            -i|--info)
                INFO_ONLY=true
                shift
                ;;
            -f|--force)
                FORCE_OVERWRITE=true
                shift
                ;;
            -*)
                log_error "Unknown option: $1"
                echo "Use --help for usage information"
                exit 1
                ;;
            *)
                if [ -z "${SHARE_URL:-}" ]; then
                    SHARE_URL="$1"
                else
                    OUTPUT_DIR="$1"
                fi
                shift
                ;;
        esac
    done

    if [ -z "${SHARE_URL:-}" ]; then
        log_error "Missing share URL"
        echo "Use --help for usage information"
        exit 1
    fi
}

# Main
main() {
    if [ $# -lt 1 ]; then
        usage
    fi

    parse_args "$@"

    check_dependencies
    parse_share_url "$SHARE_URL"
    get_session_cookie "$SHARE_URL"
    get_file_metadata

    if [ "$INFO_ONLY" = true ]; then
        log_info "Info-only mode, skipping download"
        exit 0
    fi

    download_file

    log_info "Done!"
}

main "$@"
