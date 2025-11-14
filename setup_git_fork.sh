#!/bin/bash
#
# Setup Git Fork for Lab3 Deployment
# This script helps you set up your own fork/branch
#

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "╔══════════════════════════════════════════════════════════════════════════════╗"
echo "║              Git Fork Setup for Lab3 Deployment                              ║"
echo "╚══════════════════════════════════════════════════════════════════════════════╝"
echo ""

# Check current status
echo -e "${BLUE}Current Git Configuration:${NC}"
echo "Repository: $(git remote get-url origin)"
echo "Branch: $(git branch --show-current)"
echo ""

# Warning
echo -e "${YELLOW}⚠️  WARNING:${NC}"
echo "You are currently using the course's original repository!"
echo "You should NOT push your changes there."
echo ""

# Get user's GitHub username
read -p "Enter your GitHub username: " GITHUB_USERNAME

if [ -z "$GITHUB_USERNAME" ]; then
    echo -e "${RED}Error: GitHub username is required${NC}"
    exit 1
fi

# Confirm fork exists
echo ""
echo -e "${YELLOW}Before continuing, make sure you have forked the repository:${NC}"
echo "1. Go to: https://github.com/NEWSLabNTU/AutoSDV"
echo "2. Click the 'Fork' button (top right)"
echo "3. Your fork will be at: https://github.com/${GITHUB_USERNAME}/AutoSDV"
echo ""
read -p "Have you forked the repository? (y/n) " -n 1 -r
echo ""

if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo ""
    echo "Please fork the repository first, then run this script again."
    exit 0
fi

# Step 1: Create new branch
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${BLUE}Step 1: Creating new branch 'lab3-deployment'${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if git show-ref --verify --quiet refs/heads/lab3-deployment; then
    echo -e "${YELLOW}Branch 'lab3-deployment' already exists${NC}"
    read -p "Switch to it? (y/n) " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        git checkout lab3-deployment
    fi
else
    git checkout -b lab3-deployment
    echo -e "${GREEN}✓ Created and switched to branch 'lab3-deployment'${NC}"
fi

# Step 2: Rename original remote
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${BLUE}Step 2: Renaming original remote to 'upstream'${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if git remote | grep -q "^upstream$"; then
    echo -e "${YELLOW}'upstream' remote already exists${NC}"
else
    git remote rename origin upstream
    echo -e "${GREEN}✓ Renamed 'origin' to 'upstream'${NC}"
fi

# Step 3: Add your fork as origin
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${BLUE}Step 3: Adding your fork as 'origin'${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

YOUR_FORK="https://github.com/${GITHUB_USERNAME}/AutoSDV.git"

if git remote | grep -q "^origin$"; then
    echo -e "${YELLOW}'origin' remote already exists, removing...${NC}"
    git remote remove origin
fi

git remote add origin "$YOUR_FORK"
echo -e "${GREEN}✓ Added your fork as 'origin': $YOUR_FORK${NC}"

# Step 4: Verify setup
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${BLUE}Step 4: Verifying configuration${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

echo ""
echo "Remote repositories:"
git remote -v
echo ""
echo "Current branch: $(git branch --show-current)"
echo ""

# Success
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${GREEN}✅ SETUP COMPLETE!${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Your repository is now configured:"
echo "  upstream → https://github.com/NEWSLabNTU/AutoSDV.git (course repo)"
echo "  origin   → $YOUR_FORK (your fork)"
echo ""
echo "Next steps:"
echo ""
echo "1. Stage your changes:"
echo -e "   ${BLUE}git add .${NC}"
echo ""
echo "2. Commit your changes:"
echo -e "   ${BLUE}git commit -m \"Lab3: AutoSDV containerization with all fixes\"${NC}"
echo ""
echo "3. Push to YOUR fork:"
echo -e "   ${BLUE}git push -u origin lab3-deployment${NC}"
echo ""
echo "4. Build and push Docker image:"
echo -e "   ${BLUE}cd docker && make build${NC}"
echo -e "   ${BLUE}cd .. && ./push_to_dockerhub.sh${NC}"
echo ""
echo "5. On Jetsons, clone YOUR fork:"
echo -e "   ${BLUE}git clone -b lab3-deployment $YOUR_FORK${NC}"
echo ""

