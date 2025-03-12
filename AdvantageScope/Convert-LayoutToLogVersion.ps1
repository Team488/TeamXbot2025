# Define the file paths
$inputFilePath = ".\StandardLayout.json"
$outputFilePath = ".\LoggedLayout.json"

# Read the content of the input file
$content = Get-Content -Path $inputFilePath -Raw

# Replace all instances of "NT:/AdvantageKit/" with blank strings
$modifiedContent = $content -replace "NT:/AdvantageKit/", "/"

# Write the modified content to the output file
Set-Content -Path $outputFilePath -Value $modifiedContent

Write-Host "Replacements complete. Modified content saved to $outputFilePath"