install(DIRECTORY
  config
  launch
  maps
  params   # <--- ⚠️ ต้องมีบรรทัดนี้ครับ! สำคัญมาก
  DESTINATION share/${PROJECT_NAME}
)
