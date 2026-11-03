param([Parameter(Mandatory=$true)][string]$Folder)
Add-Type -AssemblyName System.Drawing
$data = Get-Content -LiteralPath (Join-Path $Folder 'schematic_geometry.json') -Raw | ConvertFrom-Json
$xs = @($data.strips | ForEach-Object { $_.points | ForEach-Object { [double]$_[0] } })
$ys = @($data.strips | ForEach-Object { $_.points | ForEach-Object { [double]$_[1] } })
$xmin=($xs | Measure-Object -Minimum).Minimum; $xmax=($xs | Measure-Object -Maximum).Maximum
$ymin=($ys | Measure-Object -Minimum).Minimum; $ymax=($ys | Measure-Object -Maximum).Maximum
$width=1800; $height=1200
if (($xmax-$xmin) -gt 3*($ymax-$ymin)) { $height=650 }
$scale=[Math]::Min(($width-120)/($xmax-$xmin),($height-200)/($ymax-$ymin))
$ox=($width-($xmax-$xmin)*$scale)/2; $oy=100+($height-200-($ymax-$ymin)*$scale)/2
$bitmap=New-Object System.Drawing.Bitmap($width,$height)
$graphics=[System.Drawing.Graphics]::FromImage($bitmap)
$graphics.SmoothingMode=[System.Drawing.Drawing2D.SmoothingMode]::AntiAlias
$graphics.Clear([System.Drawing.Color]::FromArgb(241,245,249))
$road=New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(71,85,105))
$walk=New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(180,190,200))
$pen=New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(210,218,226),0.6)
foreach ($strip in $data.strips) {
 $points=New-Object 'System.Collections.Generic.List[System.Drawing.PointF]'
 foreach ($point in $strip.points) { $points.Add([System.Drawing.PointF]::new([single]($ox+($point[0]-$xmin)*$scale),[single]($oy+($ymax-$point[1])*$scale))) }
 if ($points.Count -lt 3) { continue }
 $brush=$walk; if ($strip.kind -eq 'driving') {$brush=$road}
 $graphics.FillPolygon($brush,$points.ToArray()); $graphics.DrawPolygon($pen,$points.ToArray())
}
$font=New-Object System.Drawing.Font('Arial',20)
$small=New-Object System.Drawing.Font('Arial',12)
$graphics.DrawString($data.name,$font,[System.Drawing.Brushes]::Black,40,22)
$graphics.DrawString('OpenDRIVE layout schematic | dark: driving lanes | light: sidewalks | north: up',$small,[System.Drawing.Brushes]::DimGray,40,65)
foreach ($label in $data.labels) {
 $px=[single]($ox+($label.x-$xmin)*$scale);$py=[single]($oy+($ymax-$label.y)*$scale)
 $graphics.FillEllipse([System.Drawing.Brushes]::White,$px-13,$py-13,26,26)
 $graphics.DrawString($label.text,$small,[System.Drawing.Brushes]::DarkBlue,$px-7,$py-10)
}
$graphics.DrawString('Source map SHA256: '+$data.sha256,$small,[System.Drawing.Brushes]::DimGray,40,($height-40))
$bitmap.Save((Join-Path (Resolve-Path $Folder).Path 'schematic.png'),[System.Drawing.Imaging.ImageFormat]::Png)
$graphics.Dispose();$bitmap.Dispose();$road.Dispose();$walk.Dispose();$pen.Dispose();$font.Dispose();$small.Dispose()
