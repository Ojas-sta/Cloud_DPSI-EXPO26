const fs = require('fs');
const { execSync } = require('child_process');

function extractTransparent(inputPath, outputPath, isDarkBg = false, cropTop = 0, cropBottom = 0, threshold = 230) {
  const probe = execSync(`ffprobe -v error -select_streams v:0 -show_entries stream=width,height -of csv=s=x:p=0 "${inputPath}"`).toString().trim();
  const [w, h] = probe.split('x').map(Number);
  console.log(`Processing ${inputPath} (${w}x${h}) -> ${outputPath}...`);

  execSync(`ffmpeg -y -i "${inputPath}" -vframes 1 -f rawvideo -pix_fmt rgba /tmp/in.raw 2>/dev/null`);
  const buf = fs.readFileSync('/tmp/in.raw');

  const visited = new Uint8Array(w * h);
  const queue = [];

  // Seed borders
  for (let x = 0; x < w; x++) {
    queue.push(x, 0);
    queue.push(x, h - 1);
  }
  for (let y = 0; y < h; y++) {
    queue.push(0, y);
    queue.push(w - 1, y);
  }

  let head = 0;
  while (head < queue.length) {
    const px = queue[head++];
    const py = queue[head++];
    const idx = py * w + px;
    if (visited[idx]) continue;
    visited[idx] = 1;

    const offset = idx * 4;
    const r = buf[offset];
    const g = buf[offset + 1];
    const b = buf[offset + 2];

    let isBg = false;
    if (isDarkBg) {
      if (r < 25 && g < 25 && b < 25) isBg = true;
    } else {
      if (r > threshold && g > threshold && b > threshold) isBg = true;
    }

    // Force crop top and bottom regions if specified (e.g. text removal)
    if (py < cropTop || py > h - cropBottom) {
      isBg = true;
    }

    if (isBg) {
      buf[offset + 3] = 0; // Transparent
      if (px > 0 && !visited[idx - 1]) queue.push(px - 1, py);
      if (px < w - 1 && !visited[idx + 1]) queue.push(px + 1, py);
      if (py > 0 && !visited[idx - w]) queue.push(px, py - 1);
      if (py < h - 1 && !visited[idx + w]) queue.push(px, py + 1);
    }
  }

  // Alpha edge refinement: 1-pixel soft antialiasing
  for (let y = 1; y < h - 1; y++) {
    for (let x = 1; x < w - 1; x++) {
      const idx = y * w + x;
      const offset = idx * 4;
      if (buf[offset + 3] > 0) {
        let transNeighbors = 0;
        if (buf[(idx - 1) * 4 + 3] === 0) transNeighbors++;
        if (buf[(idx + 1) * 4 + 3] === 0) transNeighbors++;
        if (buf[(idx - w) * 4 + 3] === 0) transNeighbors++;
        if (buf[(idx + w) * 4 + 3] === 0) transNeighbors++;
        if (transNeighbors > 0) {
          buf[offset + 3] = Math.round(255 * (1 - transNeighbors * 0.18));
        }
      }
    }
  }

  fs.writeFileSync('/tmp/out.raw', buf);
  execSync(`ffmpeg -y -f rawvideo -pix_fmt rgba -s ${w}x${h} -i /tmp/out.raw -vframes 1 -update 1 "${outputPath}" 2>/dev/null`);
  console.log(`Saved transparent cutout to ${outputPath}`);
}

const assetsDir = '/Users/roopalisingh/.gemini/antigravity/scratch/cloud-argus-docent/assets';
const uploadedDir = '/Users/roopalisingh/.gemini/antigravity/brain/ab9c06fe-2bf2-446d-becc-2e4354fd5ec7/.user_uploaded';

// 1. Walking 3/4 pose (1000x1000)
extractTransparent(
  `${uploadedDir}/media_1793457466784_99ff2a3c.png`,
  `${assetsDir}/robot-pose-walking.png`,
  false, 0, 0, 235
);

// 2. Direct Front portrait (554x554)
extractTransparent(
  `${uploadedDir}/media_1793457466795_4ad22660.png`,
  `${assetsDir}/robot-pose-front.png`,
  false, 0, 0, 235
);

// 3. Macro Head & Sensor Close-up (900x900)
extractTransparent(
  `${uploadedDir}/media_1793457486352_b2f050af.png`,
  `${assetsDir}/robot-pose-head-macro.png`,
  false, 0, 0, 235
);

// 4. Full standing pose with RealSense D435i Payload & Rail (Extracted from Poster 1, cropped text)
extractTransparent(
  `${uploadedDir}/media_1793459077389_c84152a6.jpg`,
  `${assetsDir}/robot-pose-payload-standing.png`,
  false, 250, 140, 238
);

// 5. Isometric / 3/4 aerial view
extractTransparent(
  `${uploadedDir}/media_1793457466795_e918d4e6.png`,
  `${assetsDir}/robot-pose-aerial.png`,
  false, 0, 0, 235
);

console.log('All transparent robot cutouts generated successfully!');
